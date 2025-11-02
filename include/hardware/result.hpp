#pragma once

#include <optional>
#include <utility>

namespace lemlib {

/**
 * @brief simple class for optional error handling
 *
 * @tparam T value type
 * @tparam E error type
 */
template<typename T, typename E>
class Result {
  public:
    /**
     * @brief construct a Result from a value and an error
     *
     * @tparam U value argument type
     * @tparam F error argument type
     */
    template<typename U, typename F>
    constexpr Result(U&& value, F&& error)
        : m_value(std::forward<U>(value)),
          m_error(std::forward<F>(error)) {}

    /**
     * @brief construct a Result from a value
     *
     * @tparam U value argument type
     */
    template<typename U>
    constexpr Result(U&& value)
        : m_value(std::forward<U>(value)) {}

    /**
     * @brief get the "normal" value
     *
     * @tparam Self deduced self type
     *
     * @param self this Result instance
     *
     * @return "normal" value
     */
    template<typename Self>
    constexpr auto value(this Self&& self) {
        return std::forward<Self>(self).m_value;
    }

    /**
     * @brief get the "error" value
     *
     * @tparam Self deduced self type
     *
     * @param self this Result instance
     *
     * @return "error" value
     */
    template<typename Self>
    constexpr auto error(this Self&& self) {
        return std::forward<Self>(self).m_error;
    }

    /**
     * @brief conversion operator for an l-value reference to the "normal" value type
     *
     * @return T&
     */
    constexpr operator T&() & {
        return m_value;
    }

    /**
     * @brief conversion operator for a const l-value reference to the "normal" value type
     *
     * @return const T&
     */
    constexpr operator const T&() const& {
        return m_value;
    }

    /**
     * @brief conversion operator for an r-value reference to the "normal" value type
     *
     * @return T&&
     */
    constexpr operator T&&() && {
        return std::move(m_value);
    }

  private:
    T m_value;
    std::optional<E> m_error;
};
} // namespace lemlib