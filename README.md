# MCI10797 - Rpi Zero IO

![Perspectiva](assets/perspectiva.png)

**RPI Zero IO** es una tarjeta compatible con [Raspberry Pi Zero 2 W](https://mcielectronics.cl/shop/product/raspberry-pi-zero-2-w/) preparada para inputs y outputs de alto voltaje, ideal para control y automatización. Sus 4 entradas análogas permiten un manejo de requerimiento industrial de forma ordenada, facilitando el monitoreo. También tiene 2 entradas aisladas con optoacopladores, siendo entradas digitales de hasta 24V. Por otro lado, sus 4 relés ayudan con el manejo de elementos externos de alta demanda de voltaje. Esto se complementa con un zumbador, que permite mantener a alerta los cambios que se puedan detectar.

## Características principales

- Compatible con [Raspberry Pi Zero 2 W](https://mcielectronics.cl/shop/product/raspberry-pi-zero-2-w/)
- Componentes: 4 Relés, 2 Optoacopladores, 1 Zumbador, Comunicación RS-485, entrada de voltaje (9 – 24VC), 1 Conversor digital a análogo y 4 Entradas análogas ADC conectadas por I2C.
- Conexiones: Header para Raspberry Pi Zero 2 W, terminales para el voltaje, RLY 1 – RLY4 (Relays), ADC0 – ADC3 (Inputs análogos), I1 e I2 (Inputs digitales con optoacoplador).
- Voltaje soportado: 9 – 24VDC en la entrada de voltaje, 250 VAC – 30VDC en relés
- Agujeros de montaje M2.5
- Además, incluye headers conectados directamente a pins de la Raspberry Pi Zero 2 W, pero aproximados a los componentes compatibles de la tarjeta para agilizar el cableado, mantener orden y permitir al usuario elegir si optar o no por estas.

## Pins y conexiones

![Conexiones de Rpi Zero IO](assets/conexiones.png)
![Tabla de pins RPI Zero IO](assets/tabla_pines.png)

## Requerimientos de hardware

- Computador
- Cable micro USB o fuente de voltaje
- Cables hembra-hembra
- Tarjeta microSD
- Adaptador de microSD a USB
- Raspberry Pi Zero 2 W

## Requerimientos de software

- [Raspberry Pi Imager](https://www.raspberrypi.com/software/)
- [Commix](https://xiazai.zol.com.cn/detail/27/263403.shtml)

## Configuración de la Raspberry

Antes de utilizar la tarjeta Raspberry Pi Zero 2 W, se debe configurar desde la tarjeta **microSD** con **Raspberry Pi Imager**. Empezamos seleccionando nuestro dispositivo como Raspberry Pi Zero 2 W. En Sistema Operativo (SO) tenemos que seleccionar que utilizaremos “otro sistema operativo de uso general” para poder controlar la tarjeta con nuestro computador. En la sección de almacenamiento seleccionamos nuestra tarjeta SD.  Luego solo necesitamos crear un usuario y configurar el internet al mismo que utiliza el computador desde el vamos a programar nuestros códigos. Con esto terminado, podemos insertar la tarjeta SD a la Raspberry.

Para utilizar la tarjeta necesitamos energizarla. Con la tarjeta Raspberry Pi Zero IO podemos utilizar una fuente de poder externa de 9 a 24 V. Después de menos de 3 minutos, se puede encontrar la tarjeta desde el computador en el Símbolo del Sistema (o CMD) utilizando el comando de linux `ssh remote_username@remote_host`

## Configuración de I2C y Puerto Serial

Antes de pasar a los códigos, necesitamos también **activar el protocolo I2C** (necesitado para las funciones ADC y DAC) y el **monitor serial** (para la comunicación RS485). Para esto, escribimos el comando `sudo raspi-config`, desde el cual accederemos a “Interfacing Options”. Aquí seleccionamos I2C y confirmamos que queremos activarlo. Volviendo a Interfacing Options también seleccionamos Monitor Serial, desde el cual se nos pide confirmar si se quiere activar el Login Shell, al cual seleccionas que “Si” en caso de que quieras controlar la Raspberry Pi desde un terminal del puerto serial. Si quieres usar el puerto serial para controlar otros dispositivos, selecciona “No”, como será necesario en los ejemplos. Luego, se pide confirmar que se quiera activar el puerto serial, a lo cual hay que indicar “Si”. Es necesario reiniciar la Raspberry.
Una vez la Raspberry esté encendida y conectada de nuevo, hay que instalar las utilidades de I2C tools con `sudo apt update` y `sudo apt install i2c-tools python3-smbus`.

## Cargar un código

Para escribir un código en nuestra Raspberry Pi Zero 2 W tenemos que utilizar el comando `nano NombreDelCódigo.py`, lo cual nos abrirá una pestaña para escribir el código. Es importante tener en cuenta que esto es en Python. Una vez se termine, podemos presionar Ctrl+X y guardar el código. Para cargarlo solo hay que escribir `python3 NombreDelCódigo.py`, y para interrumpirlo presionar Ctrl+C.

## Conexiones

![Conexiones en Rpi Zero IO](assets/conexiones_ejemplos.png)

## Ejemplo1: Lectura de los Optocopladores
El ejemplo lee las entradas de los optocopladores, indicando si estos están siendo activados o no.

```python
import RPi.GPIO as GPIO
import time

# ================= CONFIG =================
OPTO_1 = 23
OPTO_2 = 24

GPIO.setmode(GPIO.BCM)
GPIO.setup(OPTO_1, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(OPTO_2, GPIO.IN, pull_up_down=GPIO.PUD_UP)

print("🟢 Lectura de optoacopladores iniciada (Ctrl+C para salir)")
try:
    while True:
        estado1 = GPIO.input(OPTO_1)
        estado2 = GPIO.input(OPTO_2)
        print(f"Opto 1 (GPIO23): {'ACTIVO  ' if estado1 == 0 else 'INACTIVO'} | "
              f"Opto 2 (GPIO24): {'ACTIVO  ' if estado2 == 0 else 'INACTIVO'}")
        time.sleep(0.5)
except KeyboardInterrupt:
    print("\nSaliendo...")
finally:
    GPIO.cleanup()
```
## Ejemplo 2: Lectura ADC

Lee las entradas “Análogas a digital” conectadas por I2C.

```python
from smbus import SMBus
import time

bus = SMBus(1)
ADC_ADDR = 0x48

# Configuración base: PGA ±4.096V, single-shot, 128SPS
BASE_CONFIG = 0x0183

# MUX single-ended para cada canal
MUX = [0x4000, 0x5000, 0x6000, 0x7000]

def read_adc(channel):
    if channel < 0 or channel > 3:
        raise ValueError("Canal ADC debe ser 0-3")

    # OS=1 para iniciar conversión
    config = 0x8000 | MUX[channel] | BASE_CONFIG
    # Escribe configuración
    bus.write_i2c_block_data(ADC_ADDR, 0x01,
                             [(config >> 8) & 0xFF, config & 0xFF])
    time.sleep(0.01)
    # Lee registro de conversión
    data = bus.read_i2c_block_data(ADC_ADDR, 0x00, 2)
    value = (data[0] << 8) | data[1]
    # Ajuste para negativos
    if value > 0x7FFF:
        value -= 0x10000

    # Convierte a voltios
    voltage = value * 4.096 / 32768.0
    return voltage        # Si quieres leer el valor recibido, reemplaza voltage por value

while True:
    for ch in range(4):
        v = read_adc(ch)
        print(f"ADC{ch}: {v:.3f} V")
    print("-----")
    time.sleep(1)
```

## Ejemplo 3: Prueba Outputs

El código prueba el funcionamiento del zumbador y los relés. Un led se debe encender cada vez que su relé está activado.

```python
import RPi.GPIO as GPIO
import time

# Usamos numeración BCM (GPIO)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

outputs = {
    "RELAY1": 16,
    "RELAY2": 5,
    "RELAY3": 6,
    "RELAY4": 26,
    "BUZZER": 13
}

# Configurar pines como salida
for pin in outputs.values():
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

def test_outputs():
    print("== Prueba de Salidas ==")
    for name, pin in outputs.items():
        print(f"Activando {name}")
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(0.2)
    print()

try:
    while True:
        test_outputs()

except KeyboardInterrupt:
    print("\nSaliendo...")

finally:
    GPIO.cleanup()`

##Ejemplo 4: Aumento DAC

Este código hace que aumente y disminuye gradualmente el voltaje que sale del conversor “Digital a análogo”, también conectado por I2C.

```python
import smbus
import time

bus = smbus.SMBus(1)
DAC_ADDR = 0x60

def set_dac(value):         # Define el valor entregado por el DAC
    """
    value: 0–4095 (12 bits)
    """
    value = max(0, min(4095, value))

    msb = value >> 4              # bits 11..4
    lsb = (value & 0x0F) << 4     # bits 3..0

    # 0x40 = comando "Write DAC register"
    bus.write_i2c_block_data(DAC_ADDR, 0x40, [msb, lsb])

try:
    for value in range(0, 4096, 5):       # Aumento gradual de voltaje
        set_dac(value)
        time.sleep(0.02)

    time.sleep(1)

    for value in range(4095, -1, -5):     # Disminución gradual de voltaje
        set_dac(value)
        time.sleep(0.02)

except KeyboardInterrupt:
    print("Interrumpido")

finally:
    set_dac(0)      #Apaga el DAC
x00, [msb, lsb]
```

## Ejemplo 5

Este código utiliza la Raspberry como esclavo, mandando órdenes desde Commix. 

```python
import serial
import time
import struct
from smbus2 import SMBus

# ================= CONFIG =================
SLAVE_ID = 0x01

SERIAL_PORT = "/dev/serial0"
BAUDRATE = 9600

ADC_ADDR = 0x48      # ADS1115
DAC_ADDR = 0x60      # MCP4725

bus = SMBus(1)

# =============== RS485 ====================
ser = serial.Serial(
    port=SERIAL_PORT,
    baudrate=BAUDRATE,
    bytesize=8,
    parity='N',
    stopbits=1,
    timeout=0.1
)

# =============== CRC ======================
def calc_crc(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)

# =============== ADC ADS1115 ===============
def read_adc(channel):
    if channel > 3:
        return 0

    config = (
        0x8000 |
        (0x4000 >> channel) |
        0x0200 |
        0x0100 |
        0x0080 |
        0x0003
    )

    bus.write_i2c_block_data(
        ADC_ADDR, 0x01,
        [(config >> 8) & 0xFF, config & 0xFF]
    )
    time.sleep(0.01)

    data = bus.read_i2c_block_data(ADC_ADDR, 0x00, 2)
    return struct.unpack(">h", bytes(data))[0]

# =============== DAC MCP4725 ===============
def set_dac(value):
    value = max(0, min(4095, value))
    msb = value >> 4
    lsb = (value & 0x0F) << 4
    bus.write_i2c_block_data(DAC_ADDR, 0x40, [msb, lsb])

# =============== MENSAJE TEXTO ============
def send_text(msg):
    ser.write((msg + "\r\n").encode())

# =============== MODBUS ====================
def handle_request(req):
    if len(req) < 8:
        return None

    if req[0] != SLAVE_ID or calc_crc(req[:-2]) != req[-2:]:
        return None

    func = req[1]

    # -------- Read Input Registers (ADC) ----
    if func == 0x04:
        start, qty = struct.unpack(">HH", req[2:6])
        resp = bytearray([SLAVE_ID, 0x04, qty * 2])

        for i in range(qty):
            val = read_adc(start + i)
            resp += struct.pack(">H", val & 0xFFFF)

            # MENSAJE LEGIBLE
            send_text(f"ADC{start + i} = {val}")

        resp += calc_crc(resp)
        return resp

    # -------- Read Holding Register (DAC) ---
    elif func == 0x03:
        resp = bytearray([SLAVE_ID, 0x03, 2])
        resp += struct.pack(">H", 0)
        resp += calc_crc(resp)
        send_text("Lectura DAC solicitada")
        return resp

    # -------- Write Holding Register (DAC) --
    elif func == 0x06:
        addr, value = struct.unpack(">HH", req[2:6])
        if addr == 0:
            set_dac(value)
            send_text(f"DAC actualizado a {value}")
            resp = bytearray(req[:6])
            resp += calc_crc(resp)
            return resp

    return None

# =============== MAIN LOOP =================
print("🟢 Modbus RTU esclavo listo (ADC + DAC + texto)")

buffer = b""

while True:
    if ser.in_waiting:
        buffer += ser.read(ser.in_waiting)
        time.sleep(0.01)

        if len(buffer) >= 8:
            response = handle_request(buffer)
            if response:
                ser.write(response)
            buffer = b""
```

Para comprobar el funcionamiento, desde Commix se pueden usar los siguientes hexcode para dar distintas órdenes:
- Leer ADC0:       01 04 00 00 00 01 31 CA
- Leer los 4 ADC:       01 04 00 00 00 04 F1 C9
- Leer estado de los Relés:       01 01 00 00 00 04 3D C9
- Encender Relé 0:       01 05 00 00 FF 00 8C 3A
- Apagar Relé 0:       01 05 00 00 00 00 CD CA
- Encender Relé 0 y 2:       01 0F 00 00 00 04 01 05 5E 8A

