import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import random
import threading
import paho.mqtt.client as mqtt
import struct
import array
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
  
from gi.repository import GLib
import sys

from random import randint

# Constants for your service and characteristics
SERVICE_UUID = "8480779a-cc71-4031-b718-01987fac8f3a"
TEMP_CHAR_UUID = "84dda415-35d7-47a1-8b5d-447c1effecfa"
STEPS_CHAR_UUID = "359a072c-51d6-4532-b2d2-ae2638504185"
ACCEL_CHAR_UUID = "60ef4698-b6d2-416d-b112-66baad9bff96"
GYRO_CHAR_UUID = "eda0272c-0073-4a05-9b0b-d57017716327"
PRES_CHAR_UUID = "da833380-be08-4df0-af72-84bf06028ec4"
HUMID_CHAR_UUID = "1722444b-ccd3-4f40-b7ee-ff4197c2649f"
ALT_CHAR_UUID = "943d737f-655f-4df7-a3d1-9ca99ecc37fe"
ANGLE_CHAR_UUID = "c578768f-16ec-4beb-a34c-7207d347119f"



mainloop = None

BLUEZ_SERVICE_NAME = 'org.bluez'
GATT_MANAGER_IFACE = 'org.bluez.GattManager1'
DBUS_OM_IFACE =      'org.freedesktop.DBus.ObjectManager'
DBUS_PROP_IFACE =    'org.freedesktop.DBus.Properties'
ADAPTER_INTERFACE = 'org.bluez.Adapter1'

GATT_SERVICE_IFACE = 'org.bluez.GattService1'
GATT_CHRC_IFACE =    'org.bluez.GattCharacteristic1'
GATT_DESC_IFACE =    'org.bluez.GattDescriptor1'

# MQTT Client Setup
broker_address = "127.0.0.1"
broker_port = 1883
mqtt_client = mqtt.Client()
sensor_data = {}


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("havpi/temperature")
        client.subscribe("havpi/steps")
        client.subscribe("havpi/pressure")
        client.subscribe("havpi/accel_x")
        client.subscribe("havpi/accel_y")
        client.subscribe("havpi/accel_z")
        client.subscribe("havpi/gyro_x")
        client.subscribe("havpi/gyro_y")
        client.subscribe("havpi/gyro_z")
        client.subscribe("havpi/humidity")
        client.subscribe("havpi/altitude")
        client.subscribe("havpi/pitch_angle")
        client.subscribe("havpi/roll_angle")
    else:
        print(f"Failed to connect, return code {rc}\n")


def on_message(client, userdata, message):
    global sensor_data
    if message.topic == "havpi/temperature":
        sensor_data["Temperature"] = message.payload.decode()
        print("Temperature received: ", sensor_data["Temperature"])
    
    if message.topic == "havpi/steps":
        sensor_data["Steps"] = message.payload.decode()
        print("Steps received: ", sensor_data["Steps"])
        
    if message.topic == "havpi/accel_x":
        sensor_data["Accel_x"] = message.payload.decode()
        print("Accel_x received: ", sensor_data["Accel_x"])
        
    if message.topic == "havpi/accel_y":
        sensor_data["Accel_y"] = message.payload.decode()
        print("Accel_y received: ", sensor_data["Accel_y"])
        
    if message.topic == "havpi/accel_z":
        sensor_data["Accel_z"] = message.payload.decode()
        print("Accel_z received: ", sensor_data["Accel_z"])
        
    if message.topic == "havpi/gyro_x":
        sensor_data["Gyro_x"] = message.payload.decode()
        print("Gyro_x received: ", sensor_data["Gyro_x"])
        
    if message.topic == "havpi/gyro_y":
        sensor_data["Gyro_y"] = message.payload.decode()
        print("Gyro_y received: ", sensor_data["Gyro_y"])
        
    if message.topic == "havpi/gyro_z":
        sensor_data["Gyro_z"] = message.payload.decode()
        print("Gyro_z received: ", sensor_data["Gyro_z"])
        
    if message.topic == "havpi/pressure":
        sensor_data["Pressure"] = message.payload.decode()
        print("Pressure received: ", sensor_data["Pressure"])
        
    if message.topic == "havpi/humidity":
        sensor_data["Humidity"] = message.payload.decode()
        print("Humidity received: ", sensor_data["Humidity"])
    
    if message.topic == "havpi/altitude":
        sensor_data["Altitude"] = message.payload.decode()
        print("Altitude received: ", sensor_data["Altitude"])
        
    if message.topic == "havpi/pitch_angle":
        sensor_data["pitch_angle"] = message.payload.decode()
        print("pitch_angle received: ", sensor_data["pitch_angle"])
        
    if message.topic == "havpi/roll_angle":
        sensor_data["roll_angle"] = message.payload.decode()
        print("roll_angle received: ", sensor_data["roll_angle"])

    
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_message



class InvalidArgsException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.freedesktop.DBus.Error.InvalidArgs'

class NotSupportedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotSupported'

class NotPermittedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.NotPermitted'

class InvalidValueLengthException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.InvalidValueLength'

class FailedException(dbus.exceptions.DBusException):
    _dbus_error_name = 'org.bluez.Error.Failed'
    
    
class Application(dbus.service.Object):
    """
    org.bluez.GattApplication1 interface implementation
    """
    def __init__(self, bus):
        self.path = '/'
        self.services = []
        dbus.service.Object.__init__(self, bus, self.path)
        self.add_service(HAVSensorService(bus, 0))
        #self.add_service(BatteryService(bus, 1))
        #self.add_service(TestService(bus, 2))

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_service(self, service):
        self.services.append(service)

    @dbus.service.method(DBUS_OM_IFACE, out_signature='a{oa{sa{sv}}}')
    def GetManagedObjects(self):
        response = {}
        print('GetManagedObjects')

        for service in self.services:
            response[service.get_path()] = service.get_properties()
            chrcs = service.get_characteristics()
            for chrc in chrcs:
                response[chrc.get_path()] = chrc.get_properties()
                descs = chrc.get_descriptors()
                for desc in descs:
                    response[desc.get_path()] = desc.get_properties()

        return response


class Service(dbus.service.Object):
    """
    org.bluez.GattService1 interface implementation
    """
    PATH_BASE = '/org/bluez/example/service'

    def __init__(self, bus, index, uuid, primary):
        self.path = self.PATH_BASE + str(index)
        self.bus = bus
        self.uuid = uuid
        self.primary = primary
        self.characteristics = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_SERVICE_IFACE: {
                        'UUID': self.uuid,
                        'Primary': self.primary,
                        'Characteristics': dbus.Array(
                                self.get_characteristic_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_characteristic(self, characteristic):
        self.characteristics.append(characteristic)

    def get_characteristic_paths(self):
        result = []
        for chrc in self.characteristics:
            result.append(chrc.get_path())
        return result

    def get_characteristics(self):
        return self.characteristics

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_SERVICE_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_SERVICE_IFACE]


class Characteristic(dbus.service.Object):
    """
    org.bluez.GattCharacteristic1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, service):
        self.path = service.path + '/char' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.service = service
        self.flags = flags
        self.descriptors = []
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_CHRC_IFACE: {
                        'Service': self.service.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                        'Descriptors': dbus.Array(
                                self.get_descriptor_paths(),
                                signature='o')
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    def add_descriptor(self, descriptor):
        self.descriptors.append(descriptor)

    def get_descriptor_paths(self):
        result = []
        for desc in self.descriptors:
            result.append(desc.get_path())
        return result

    def get_descriptors(self):
        return self.descriptors

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_CHRC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_CHRC_IFACE]

    @dbus.service.method(GATT_CHRC_IFACE,
                        in_signature='a{sv}',
                        out_signature='ay')
    def ReadValue(self, options):
        print('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StartNotify(self):
        print('Default StartNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_CHRC_IFACE)
    def StopNotify(self):
        print('Default StopNotify called, returning error')
        raise NotSupportedException()

    @dbus.service.signal(DBUS_PROP_IFACE,
                         signature='sa{sv}as')
    def PropertiesChanged(self, interface, changed, invalidated):
        pass


class Descriptor(dbus.service.Object):
    """
    org.bluez.GattDescriptor1 interface implementation
    """
    def __init__(self, bus, index, uuid, flags, characteristic):
        self.path = characteristic.path + '/desc' + str(index)
        self.bus = bus
        self.uuid = uuid
        self.flags = flags
        self.chrc = characteristic
        dbus.service.Object.__init__(self, bus, self.path)

    def get_properties(self):
        return {
                GATT_DESC_IFACE: {
                        'Characteristic': self.chrc.get_path(),
                        'UUID': self.uuid,
                        'Flags': self.flags,
                }
        }

    def get_path(self):
        return dbus.ObjectPath(self.path)

    @dbus.service.method(DBUS_PROP_IFACE,
                         in_signature='s',
                         out_signature='a{sv}')
    def GetAll(self, interface):
        if interface != GATT_DESC_IFACE:
            raise InvalidArgsException()

        return self.get_properties()[GATT_DESC_IFACE]

    @dbus.service.method(GATT_DESC_IFACE,
                        in_signature='a{sv}',
                        out_signature='ay')
    def ReadValue(self, options):
        print ('Default ReadValue called, returning error')
        raise NotSupportedException()

    @dbus.service.method(GATT_DESC_IFACE, in_signature='aya{sv}')
    def WriteValue(self, value, options):
        print('Default WriteValue called, returning error')
        raise NotSupportedException()


class HAVSensorService(Service):
    

    def __init__(self, bus, index):
        Service.__init__(self, bus, index, SERVICE_UUID, True)
        self.add_characteristic(HAVTempChar(bus, 0, self))
        self.add_characteristic(HAVStepChar(bus, 1, self))
        self.add_characteristic(HAVAccelChar(bus, 2, self))
        self.add_characteristic(HAVGyroChar(bus, 3, self))
        self.add_characteristic(HAVPresChar(bus, 4, self))
        self.add_characteristic(HAVHumidChar(bus, 5, self))
        self.add_characteristic(HAVAltChar(bus, 6, self))
        self.add_characteristic(HAVAngleChar(bus, 7, self))
        
        self.energy_expended = 0
        
 
class HAVTempChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                TEMP_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update_temperature(self):
        print('Update Temperature')
        if not self.notifying:
            return
        print('ttt')
        GLib.timeout_add_seconds(5, self.temp_measure)
        #self.temp_measure(self)
        print('testtt')
        
        
    def temp_measure(self):
        global sensor_data
        print('temp_measure test')
        value = []
        
        for char in sensor_data["Temperature"]:
            c = ord(char)
            value.append(dbus.Byte(c))
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value }, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update_temperature()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update_temperature()
        
class HAVStepChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                STEPS_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update Steps')
        if not self.notifying:
            return
        print('steps')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test steps success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('temp_measure test')
        value = []
        
        
        for char in sensor_data["Steps"]:
            c = ord(char)
            value.append(dbus.Byte(c))
            
        
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()
        

class HAVAccelChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                ACCEL_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update accelx')
        if not self.notifying:
            return
        print('accelx')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test accelx success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('accelx_measure test')
        value = []
        
        
        for i, char in enumerate(sensor_data["Accel_x"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
            
        for i, char in enumerate(sensor_data["Accel_y"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
        
        for i, char in enumerate(sensor_data["Accel_z"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
            
        
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()
    
class HAVGyroChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                GYRO_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update gyro')
        if not self.notifying:
            return
        print('gyro')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test gyro success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('gyro test')
        value = []
        
        
        for i, char in enumerate(sensor_data["Gyro_x"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
            
        for i, char in enumerate(sensor_data["Gyro_y"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
        
        for i, char in enumerate(sensor_data["Gyro_z"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
            
        
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()    
        
class HAVPresChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                PRES_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update Pressure')
        if not self.notifying:
            return
        print('pressure')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test pressure success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('pressure test')
        value = []
        
        
        for char in sensor_data["Pressure"]:
            c = ord(char)
            value.append(dbus.Byte(c))
            
        
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()
        
        
class HAVHumidChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                HUMID_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update humidity')
        if not self.notifying:
            return
        print('humidity')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test humidity success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('humidity test')
        value = []
        
        
        for char in sensor_data["Humidity"]:
            c = ord(char)
            value.append(dbus.Byte(c))
            
        
        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()
        
        
class HAVAltChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                ALT_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update altitude')
        if not self.notifying:
            return
        print('altitude')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test altitude success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('altitude test')
        value = []
        
        
        for i, char in enumerate(sensor_data["Altitude"]):
            if i >= 6:
                break
            c = ord(char)
            value.append(dbus.Byte(c))

        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()    
        
        
class HAVAngleChar(Characteristic):
    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                ANGLE_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update(self):
        print('Update angles')
        if not self.notifying:
            return
        print('angles')
        GLib.timeout_add_seconds(5, self.measure)
        #self.temp_measure(self)
        print('test angles success')
        
        
    def measure(self):
        print('other test')
        global sensor_data
        print('angles test')
        value = []
        
        
        for i, char in enumerate(sensor_data["pitch_angle"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))
            
        for i, char in enumerate(sensor_data["roll_angle"]):
            if i >= 7:
                break
            c = ord(char)
            value.append(dbus.Byte(c))

        
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value}, [])
        #self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': dbus.Array(value, variant_level=1) }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update()    

def register_app_cb():
    print('GATT application registered')

def register_app_error_cb(error):
    print('Failed to register application: ' + str(error))
    mainloop.quit()

def find_adapter(bus):
    remote_om = dbus.Interface(bus.get_object(BLUEZ_SERVICE_NAME, '/'),
                               DBUS_OM_IFACE)
    objects = remote_om.GetManagedObjects()

    for o, props in objects.items():
        if GATT_MANAGER_IFACE in props.keys():
            return o

    return None


def subscribe_loop():
    mqtt_client.connect(broker_address, port=broker_port)
    mqtt_client.loop_forever()    
        
def main():
    global mainloop

    dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)

    bus = dbus.SystemBus()

    adapter = find_adapter(bus)
    if not adapter:
        print('GattManager1 interface not found')
        return

    service_manager = dbus.Interface(
            bus.get_object(BLUEZ_SERVICE_NAME, adapter),
            GATT_MANAGER_IFACE)

    app = Application(bus)

    mainloop = GLib.MainLoop()

    print('Registering GATT application...')

    service_manager.RegisterApplication(app.get_path(), {},
                                    reply_handler=register_app_cb,
                                    error_handler=register_app_error_cb)

    mainloop.run()

if __name__ == '__main__':
    
    mqtt_thread = threading.Thread(target=subscribe_loop)
    mqtt_thread.start()
    
    main_thread = threading.Thread(target=main)
    main_thread.start()


   


