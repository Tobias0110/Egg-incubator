# Egg-incubator
Uses an arduino nano with the DHT22 temperature and humidity sensor as an egg incubator. Using standard arduino functions for the GPIO-Ports coused strange behavior. The reason for that is unknown. Please enlighten me.

Ported the project to an ARM Cortex microcontroller from STM (stm32l031) and used the I2C-Sensor MCP9808. Now everything works fine.
