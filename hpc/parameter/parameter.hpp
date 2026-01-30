#ifndef PARAMETER_HPP_
#define PARAMETER_HPP_

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace param
{
/// @brief Parameter system fault codes
enum Fault : uint8_t
{
    NO_FAULT = 0U,
    ID_MISMATCH,
    TYPE_MISMATCH,
    OUTBUF_TOO_SMALL,
    INBUF_TOO_SMALL,
    NONCONTIGUOUS_REGISTRATION,
    TOO_MANY_PARAMETERS,
    NO_SUCH_PARAM,
    UNSUPPORTED_TYPE,
    NUM_FAULTS
};

/// @brief Fault code to string table
/// @param fault
/// @return
constexpr const char* FaultToStr(Fault fault)
{
    switch (fault)
    {
    case Fault::NO_FAULT:
    {
        return "NO_FAULT";
    }
    case Fault::ID_MISMATCH:
    {
        return "ID_MISMATCH";
    }
    case Fault::TYPE_MISMATCH:
    {
        return "TYPE_MISMATCH";
    }
    case Fault::OUTBUF_TOO_SMALL:
    {
        return "OUTBUF_TOO_SMALL";
    }
    case Fault::INBUF_TOO_SMALL:
    {
        return "INBUF_TOO_SMALL";
    }
    case Fault::NONCONTIGUOUS_REGISTRATION:
    {
        return "NONCONTIGUOUS_REGISTRATION";
    }
    case Fault::TOO_MANY_PARAMETERS:
    {
        return "TOO_MANY_PARAMETERS";
    }
    case Fault::NO_SUCH_PARAM:
    {
        return "NO_SUCH_PARAM";
    }
    case Fault::UNSUPPORTED_TYPE:
    {
        return "UNSUPPORTED_TYPE";
    }
    default:
    {
        return "";
    }
    }
}

/// @brief Parameter payload type IDs
enum TypeID : uint8_t
{
    FLOAT32 = 0U,
    UINT8,
    UINT16,
    UINT32,
    INT8,
    INT16,
    INT32,
};

/// @brief Type string lookup table
constexpr const char* TypeIDToStr(TypeID typeID)
{
    switch (typeID)
    {
    case TypeID::FLOAT32:
        return "FLOAT32";
    case TypeID::UINT8:
        return "UINT8";
    case TypeID::UINT16:
        return "UINT16";
    case TypeID::UINT32:
        return "UINT32";
    case TypeID::INT8:
        return "INT8";
    case TypeID::INT16:
        return "INT16";
    case TypeID::INT32:
        return "INT32";
    default:
        return "";
    }
}

/// @brief Parameter payload type
union Type
{
    float    _float32;
    uint8_t  _uint8;
    uint16_t _uint16;
    uint32_t _uint32;
    uint8_t _int8;
    uint16_t _int16;
    uint32_t _int32;
};

/// @brief Templated assert that fails only when instantiated
/// @tparam
template <typename>
inline constexpr bool always_false = false;

/// @brief Parameter index type
typedef uint16_t ParamIndex;

/// @brief Forward-declare parameter ID enum
enum class ParameterID : ParamIndex;

/// @brief Parameter list class
class Parameter;
class ParameterList
{
   public:
    ParameterList() {}
    static ParameterList& Inst()
    {
        static ParameterList inst;
        return inst;
    }

    /// @brief Maximum number of parameters this list can hold
    static constexpr ParamIndex maxParameters = 128U;

    /// @brief Register a parameter
    /// @param param
    /// @return param::Fault
    [[nodiscard]] Fault Register(Parameter* param);

    /// @brief Set a parameter value
    /// @param id
    /// @param value
    /// @return param::Fault
    template <typename T>
    [[nodiscard]] Fault Set(ParameterID id, T value);

    /// @brief Set a parameter value
    /// @param id
    /// @param value
    /// @return param::Fault
    [[nodiscard]] Fault Set(ParameterID id, Type value);

    /// @brief Get a parameter value
    /// @param id
    /// @param value
    /// @return param::Fault
    template <typename T>
    [[nodiscard]] Fault Get(ParameterID id, T& value) const;

    /// @brief Get a parameter value
    /// @param id
    /// @param value
    /// @return param::Fault
    [[nodiscard]] Fault Get(ParameterID id, Type& value) const;

    /// @brief Get a default parameter value
    /// @param id
    /// @param defaultValue
    /// @return param::Fault
    [[nodiscard]] Fault GetDefault(ParameterID id, Type& defaultValue) const;

    /// @brief Get a parameter type ID
    /// @param id
    /// @param type
    /// @return param::Fault
    [[nodiscard]] Fault GetType(ParameterID id, TypeID& type) const;

    /// @brief Get a parameter name
    /// @param id
    /// @param name
    /// @return param::Fault
    [[nodiscard]] Fault GetName(ParameterID id, const char*& name) const;

    /// @brief Get a parameter description
    /// @param id
    /// @param desc
    /// @return param::Fault
    [[nodiscard]] Fault GetDesc(ParameterID id, const char*& desc) const;

    /// @brief Get number of parameters
    /// @return uint16_t
    ParamIndex GetNumParams() const
    {
        return _numParameters;
    };

    /// @brief Get projected serialized size
    /// @return uint16_t
    uint16_t GetSize() const;

    /// @brief Serialize parameter list
    /// @param outBuf
    /// @param outSize
    /// @param writeSize
    /// @return param::Fault
    [[nodiscard]] Fault Serialize(uint8_t* outBuf, uint16_t outSize, uint16_t& writeSize);

    /// @brief Deserialize parameter list
    /// @param inBuf
    /// @param inSize
    /// @param readSize
    /// @return param::Fault
    [[nodiscard]] Fault Deserialize(uint8_t* inBuf, uint16_t inSize, uint16_t& readSize);

   private:
    /// @brief Number of registered parameters
    ParamIndex _numParameters = 0U;
    /// @brief Parameters
    Parameter* _parameters[maxParameters];
};

/// @brief Parameter payload
struct ParameterPayload
{
    /// @brief Parameter unique identifier
    ParameterID id;
    /// @brief Parameter payload type identifier
    TypeID typeID;
    /// @brief Current parameter value
    Type currentValue;

    /// @brief Serialize a parameter to a buffer
    /// @param outBuf output buffer
    /// @param outSize output buffer size
    /// @return param::Fault
    [[nodiscard]] Fault Serialize(uint8_t* outBuf, uint16_t outSize)
    {
        Fault fault = Fault::NO_FAULT;

        // Compute serialized size
        uint16_t param_size = sizeof(ParameterPayload);

        // See if it'll fit
        if (param_size <= outSize)
        {
            memcpy(outBuf, this, param_size);
        }
        else
        {
            fault = Fault::OUTBUF_TOO_SMALL;
        }

        return fault;
    }

    /// @brief Deserialize a parameter from a buffer
    /// @param inBuf input buffer
    /// @param inSize input buffer size
    /// @return param::Fault
    [[nodiscard]] Fault Deserialize(uint8_t* inBuf, uint16_t inSize)
    {
        Fault fault = Fault::NO_FAULT;

        // Compute expected serialized size
        uint16_t param_size = sizeof(ParameterPayload);

        // See if incoming is large enough
        if (inSize >= param_size)
        {
            static ParameterPayload staged;
            memcpy(&staged, inBuf, param_size);

            // Check if param IDs and types match
            if (staged.id != id)
            {
                fault = Fault::ID_MISMATCH;
            }
            else if (staged.typeID != typeID)
            {
                fault = Fault::TYPE_MISMATCH;
            }
            else
            {
                *this = staged;
            }
        }
        else
        {
            fault = INBUF_TOO_SMALL;
        }

        return fault;
    }

    ParameterPayload() {}
    /// @brief Construct a ParameterPayload
    /// @param id
    /// @param typeID
    /// @param currentValue
    ParameterPayload(ParameterID id, TypeID typeID, Type currentValue)
        : id(id), typeID(typeID), currentValue(currentValue)
    {
    }
};

/// @brief Full parameter structure with metadata
struct Parameter : ParameterPayload
{
    /// @brief Maximum parameter name size
    static constexpr uint16_t maxParamNameSize = 64U;
    /// @brief Maximum parameter description size
    static constexpr uint16_t maxParamDescSize = 128U;

    /// @brief Default parameter value
    Type defaultValue;
    /// @brief Parameter name
    char name[maxParamNameSize] = {0};
    /// @brief Parameter description
    char desc[maxParamDescSize] = {0};

    Parameter() {}
    /// @brief Construct and register a Parameter with ParameterList
    /// @param id
    /// @param typeID
    /// @param defaultValue
    /// @param _name
    /// @param _desc
    Parameter(ParameterID id, TypeID typeID, Type defaultValue, const char* _name,
              const char* _desc)
        : ParameterPayload(id, typeID, defaultValue), defaultValue(defaultValue)
    {
        std::memcpy(name, _name, std::min(std::strlen(_name), sizeof(name) - 1));
        std::memcpy(desc, _desc, std::min(std::strlen(_desc), sizeof(desc) - 1));

        // Register Parameter with ParameterList
        if (ParameterList::Inst().Register(this) != Fault::NO_FAULT)
        {
            // TODO: Handle
        }
    }
};

// Unique parameter name generator
#define UNIQUE_NAME2(base, count) base##count
#define UNIQUE_NAME(base, count) UNIQUE_NAME2(base, count)

// Parameter shorthand
#define DEFINE_PARAMETER(id, typeID, defaultValue, name, desc) \
    static param::Parameter UNIQUE_NAME(_reg_, __COUNTER__) =  \
        param::Parameter(id, typeID, defaultValue, name, desc);

template <typename T>
[[nodiscard]] Fault ParameterList::Set(ParameterID id, T value)
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        if constexpr (std::is_same_v<T, float>)
        {
            if (_parameters[index]->typeID == TypeID::FLOAT32)
            {
                _parameters[index]->currentValue._float32 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint8_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT8)
            {
                _parameters[index]->currentValue._uint8 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint16_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT16)
            {
                _parameters[index]->currentValue._uint16 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint32_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT32)
            {
                _parameters[index]->currentValue._uint32 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int8_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT8)
            {
                _parameters[index]->currentValue._int8 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int16_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT16)
            {
                _parameters[index]->currentValue._int16 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int32_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT32)
            {
                _parameters[index]->currentValue._int32 = value;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else
        {
            static_assert(always_false<T>, "Unsupported type");
        }
    }

    return fault;
}

template <typename T>
[[nodiscard]] Fault ParameterList::Get(ParameterID id, T& value) const
{
    Fault fault = Fault::NO_FAULT;

    ParamIndex index = static_cast<ParamIndex>(id);

    if (index >= _numParameters || index >= maxParameters)
    {
        fault = Fault::NO_SUCH_PARAM;
    }
    else
    {
        if constexpr (std::is_same_v<T, float>)
        {
            if (_parameters[index]->typeID == TypeID::FLOAT32)
            {
                value = _parameters[index]->currentValue._float32;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint8_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT8)
            {
                value = _parameters[index]->currentValue._uint8;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint16_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT16)
            {
                value = _parameters[index]->currentValue._uint16;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, uint32_t>)
        {
            if (_parameters[index]->typeID == TypeID::UINT32)
            {
                value = _parameters[index]->currentValue._uint32;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int8_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT8)
            {
                value = _parameters[index]->currentValue._int8;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int16_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT16)
            {
                value = _parameters[index]->currentValue._int16;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else if constexpr (std::is_same_v<T, int32_t>)
        {
            if (_parameters[index]->typeID == TypeID::INT32)
            {
                value = _parameters[index]->currentValue._int32;
            }
            else
            {
                fault = Fault::TYPE_MISMATCH;
            }
        }
        else
        {
            static_assert(always_false<T>, "Unsupported type");
        }
    }

    return fault;
}
}  // namespace param

#endif
