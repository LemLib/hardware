#pragma once

#include <cstdint>

namespace lemlib {

struct DynamicPort {};

constexpr DynamicPort runtime_check_port {};

class SmartPort {
    public:
        consteval SmartPort(std::int64_t port)
            : m_port(port) {
            // XXX: this doesnt actaully throw an exception since the ctor
            // is consteval, it halts compilation instead
            if (port < 1 || port > 21) throw "Port out of range!";
        }

        SmartPort(std::int64_t port, DynamicPort)
            : m_port(port) {
            if (port < 1 || port > 21) m_port = 0;
        }

        std::uint8_t get_port() const { return m_port; }

        operator std::uint8_t() const { return m_port; }
    private:
        std::uint8_t m_port;
};

class ReversibleSmartPort {
    public:
        consteval ReversibleSmartPort(std::int64_t port)
            : m_port(port < 0 ? -port : port),
              m_reversed(port < 0) {
            if (port < 0) port = -port;
            // XXX: this doesnt actaully throw an exception since the ctor
            // is consteval, it halts compilation instead
            if (port < 1 || port > 21) throw "Port out of range!";
        }

        ReversibleSmartPort(std::int64_t port, DynamicPort)
            : m_port(port < 0 ? -port : port),
              m_reversed(port < 0) {
            if (port < 0) port = -port;
            if (port < 1 || port > 21) m_port = 0;
        }

        std::uint8_t get_port() const { return m_port; }

        bool get_reversed() const { return m_reversed; }

        operator int() const { return m_port; }
    private:
        std::uint8_t m_port;
        bool m_reversed;
};

class ADIPort {
    public:
        consteval ADIPort(std::int64_t port)
            : m_port(0) {
            if (port >= 'a' && port <= 'h') port -= ('a' - 1);
            else if (port >= 'A' && port <= 'H') port -= ('A' - 1);
            if (port < 1 || port > 8) throw "Port out of range!";
            m_port = port;
        }

        ADIPort(std::int64_t port, DynamicPort)
            : m_port(0) {
            if (port < 1 || port > 21) m_port = 0;
            if (port >= 'a' && port <= 'h') port -= ('a' - 1);
            else if (port >= 'A' && port <= 'H') port -= ('A' - 1);
            if (port < 1 || port > 8) return;
            m_port = port;
        }

        std::uint8_t get_port() const { return m_port; }

        operator std::uint8_t() const { return m_port; }
    private:
        std::uint8_t m_port;
};

} // namespace lemlib
