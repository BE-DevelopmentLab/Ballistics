#pragma once
struct TwoWire { void begin() {} void setClock(unsigned long) {} };
inline TwoWire Wire;
