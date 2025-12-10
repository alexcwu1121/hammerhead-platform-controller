#include "parameter.hpp"

namespace param
{
[[nodiscard]] Fault ParameterList::Register(Parameter* param)
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(param->id);

    if (index >= maxParameters)
    {
        fault = Fault::TOO_MANY_PARAMETERS;
    }
    else if (index != _numParameters)
    {
        // Only permit contiguous registration
        fault = Fault::NONCONTIGUOUS_REGISTRATION;
    }
    else
    {
        _parameters[index] = param;
        _numParameters++;
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::GetDefault(ParameterID id, Type& defaultValue) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        defaultValue = _parameters[index]->defaultValue;
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::GetType(ParameterID id, TypeID& type) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        type = _parameters[index]->typeID;
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::GetName(ParameterID id, const char*& name) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        name = _parameters[index]->name;
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::GetDesc(ParameterID id, const char*& desc) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        desc = _parameters[index]->desc;
    }

    return fault;
}

uint16_t ParameterList::GetSize() const
{
    return _numParameters * sizeof(ParameterPayload);
}

[[nodiscard]] Fault ParameterList::Serialize(uint8_t* outBuf, uint16_t outSize, uint16_t& writeSize)
{
    Fault fault = Fault::NO_FAULT;

    writeSize = 0U;
    for (ParamIndex i = 0U; i < _numParameters; i++)
    {
        fault = _parameters[i]->Serialize(outBuf + writeSize, outSize - writeSize);

        if (fault != Fault::NO_FAULT)
        {
            break;
        }

        writeSize += sizeof(ParameterPayload);
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::Deserialize(uint8_t* inBuf, uint16_t inSize, uint16_t& readSize)
{
    Fault fault = Fault::NO_FAULT;

    readSize = 0U;
    for (ParamIndex i = 0U; i < _numParameters; i++)
    {
        fault = _parameters[i]->Deserialize(inBuf + readSize, inSize - readSize);

        if (fault != Fault::NO_FAULT)
        {
            break;
        }

        readSize += sizeof(ParameterPayload);
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::Set(ParameterID id, Type value)
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        _parameters[index]->currentValue = value;
    }

    return fault;
}

[[nodiscard]] Fault ParameterList::Get(ParameterID id, Type& value) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        value = _parameters[index]->currentValue;
    }

    return fault;
}
}  // namespace param
