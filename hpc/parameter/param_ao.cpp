#include "param_ao.hpp"

#include "cli_ao.hpp"

param::ParamAO::ParamAO()
    : QP::QActive(&initial),
      _eeprom(P_EEPROM_CS_GPIO_Port, P_EEPROM_CS_Pin, P_EEPROM_WP_GPIO_Port, P_EEPROM_WP_Pin,
              &hspi2)
{
}

void param::ParamAO::Start(const QP::QPrioSpec priority)
{
    this->start(priority,      // QP prio. of the AO
                _queue,        // event queue storage
                _queueSize,    // queue size [events]
                nullptr, 0U);  // no stack storage
    _isStarted = true;
}

void param::ParamAO::SetParam(ParameterID id, Type value)
{
    if (_isStarted)
    {
        SetParamValueEvt* evt = Q_NEW(SetParamValueEvt, PrivateSignals::SET_PARAM_VALUE_SIG);
        evt->id               = id;
        evt->value            = value;
        POST(evt, this);
    }
}

void param::ParamAO::RequestUpdate(ParameterID id)
{
    if (_isStarted)
    {
        RequestUpdateEvt* evt = Q_NEW(RequestUpdateEvt, PrivateSignals::REQUEST_UPDATE_SIG);
        evt->id               = id;
        POST(evt, this);
    }
}

void param::ParamAO::Commit()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::COMMIT_SIG);
        POST(&evt, this);
    }
}

void param::ParamAO::List()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::LIST_SIG);
        POST(&evt, this);
    }
}

void param::ParamAO::ResetToDefaults()
{
    if (_isStarted)
    {
        static QP::QEvt evt(PrivateSignals::RESET_TO_DEFAULTS_SIG);
        POST(&evt, this);
    }
}

Q_STATE_DEF(param::ParamAO, initial)
{
    Q_UNUSED_PAR(e);
    _eeprom.EnableWriteProtect();
    return tran(&active);
}

Q_STATE_DEF(param::ParamAO, active)
{
    QP::QState status_;
    switch (e->sig)
    {
    case Q_ENTRY_SIG:
    {
        // Get expected serialized size of parameter table
        uint16_t read_size = ParameterList::Inst().GetSize();
        // Read parameters from EEPROM
        _eeprom.Read(_rxBuf, read_size, _paramBlockAddr);
        // Deserialize into parameter table
        uint16_t     deserialize_size;
        param::Fault fault =
            ParameterList::Inst().Deserialize(_rxBuf, _rxBufSize, deserialize_size);

        if (fault != param::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter deserialize fault (%u)", fault);
        }
        else if (read_size != deserialize_size)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter unexpected read size");
        }

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::SET_PARAM_VALUE_SIG:
    {
        // Set a parameter value
        ParameterID id    = Q_EVT_CAST(SetParamValueEvt)->id;
        Type        value = Q_EVT_CAST(SetParamValueEvt)->value;
        Fault       fault = ParameterList::Inst().Set(id, value);
        if (fault != param::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter set fault (%u)", fault);
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::REQUEST_UPDATE_SIG:
    {
        // Publish a parameter update event
        bsp::ParameterUpdateEvt* evt =
            Q_NEW(bsp::ParameterUpdateEvt, bsp::PublicSignals::PARAMETER_UPDATE_EVT);
        evt->id     = Q_EVT_CAST(RequestUpdateEvt)->id;
        Fault fault = ParameterList::Inst().Get(evt->id, evt->value);
        if (fault != param::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter get fault (%u)", fault);
            // Garbage collect the unpublished event
            QP::QF::gc(evt);
        }
        else
        {
            PUBLISH(evt, this);
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::COMMIT_SIG:
    {
        // Write parameters to EEPROM
        uint16_t     size;
        param::Fault fault = ParameterList::Inst().Serialize(_txBuf, _txBufSize, size);
        if (fault != param::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
        }

        // Write...
        _eeprom.DisableWriteProtect();
        _eeprom.Write(_txBuf, size, _paramBlockAddr);
        _eeprom.EnableWriteProtect();
        // ...and read back
        _eeprom.Read(_rxBuf, size, _paramBlockAddr);

        // Deserialize into parameter table
        fault = ParameterList::Inst().Deserialize(_rxBuf, _rxBufSize, size);

        if (fault != param::Fault::NO_FAULT)
        {
            cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
        }

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::LIST_SIG:
    {
        for (ParamIndex i = 0U; i < ParameterList::Inst().GetNumParams(); i++)
        {
            Fault       fault;
            ParameterID id = static_cast<ParameterID>(i);
            TypeID      typeID;
            Type        value;
            Type        defaultValue;
            const char* name;
            const char* desc;
            fault = ParameterList::Inst().GetType(id, typeID);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
            fault = ParameterList::Inst().Get(id, value);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
            fault = ParameterList::Inst().GetDefault(id, defaultValue);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
            fault = ParameterList::Inst().GetName(id, name);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
            fault = ParameterList::Inst().GetDesc(id, desc);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }

            // Print header
            cli::CLIAO::Inst().Printf(
                "(%u) %s\n\r"
                "\tDesc: %s\n\r"
                "\tType: %s",
                i, name, desc, param::TypeIDToStr(typeID));

            // Print current and default values
            switch (typeID)
            {
            case TypeID::FLOAT32:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %.4f\n\r\tDefault value: %.4f\n\r",
                                          value._float32, defaultValue._float32);
                break;
            }
            case TypeID::UINT8:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._uint8, defaultValue._uint8);
                break;
            }
            case TypeID::UINT16:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._uint16, defaultValue._uint16);
                break;
            }
            case TypeID::UINT32:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._uint32, defaultValue._uint32);
                break;
            }
            case TypeID::INT8:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._int8, defaultValue._int8);
                break;
            }
            case TypeID::INT16:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._int16, defaultValue._int16);
                break;
            }
            case TypeID::INT32:
            {
                cli::CLIAO::Inst().Printf("\tCurrent value: %u\n\r\tDefault value: %u\n\r",
                                          value._int32, defaultValue._int32);
                break;
            }
            }
        }

        status_ = Q_RET_HANDLED;
        break;
    }
    case PrivateSignals::RESET_TO_DEFAULTS_SIG:
    {
        for (ParamIndex i = 0U; i < ParameterList::Inst().GetNumParams(); i++)
        {
            Fault       fault;
            ParameterID id = static_cast<ParameterID>(i);
            Type        defaultValue;
            fault = ParameterList::Inst().GetDefault(id, defaultValue);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
            fault = ParameterList::Inst().Set(id, defaultValue);
            if (fault != param::Fault::NO_FAULT)
            {
                cli::CLIAO::Inst().Printf("ERROR: Parameter fault (%u)", fault);
                continue;
            }
        }
        status_ = Q_RET_HANDLED;
        break;
    }
    default:
    {
        status_ = super(&top);
        break;
    }
    }
    return status_;
}

Q_STATE_DEF(param::ParamAO, error)
{
    QP::QState status_;
    switch (e->sig)
    {
    default:
    {
        status_ = super(&top);
        break;
    }
    }
    return status_;
}
