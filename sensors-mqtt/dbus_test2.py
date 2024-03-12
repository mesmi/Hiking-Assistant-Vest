import dbus
import dbus.exceptions
import dbus.mainloop.glib
import dbus.service
import random
import threading
import paho.mqtt.client as mqtt
import array
try:
  from gi.repository import GObject
except ImportError:
  import gobject as GObject
import sys

from random import randint

# Constants for your service and characteristics
SERVICE_UUID = "12345678-1234-5678-1234-56789abcdef0"
TEMP_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef1"
STEPS_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef2"
PRESSURE_CHAR_UUID = "12345678-1234-5678-1234-56789abcdef3"


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
temp_data = {}
step_data = {}
pres_data = {}

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT Broker!")
        client.subscribe("havpi/temperature")
        client.subscribe("havpi/steps")
        client.subscribe("havpi/pressure")
    else:
        print(f"Failed to connect, return code {rc}\n")


def on_message(client, userdata, message):
    global temp_data
    global step_data
    global pres_data
    if message.topic == "havpi/temperature":
        temp_data["Data"] = message.payload.decode()
        print("Message received: ", temp_data)
    
    if message.topic == "havpi/steps":
        step_data["Data"] = message.payload.decode()
        print("Message received: ", step_data)
    
    if message.topic == "havpi/pressure":
        pres_data["Data"] = message.payload.decode()
        print("Message received: ", pres_data)
    
    #sensor_data["data1"] = message.payload.decode()
    #print("Message received: ", sensor_data["data1"])
    
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
    """
    Fake Heart Rate Service that simulates a fake heart beat and control point
    behavior.

    """
    #HR_UUID = '0000180d-0000-1000-8000-00805f9b34fb'

    def __init__(self, bus, index):
        Service.__init__(self, bus, index, SERVICE_UUID, True)
        self.add_characteristic(HAVTempChar(bus, 0, self))
        self.add_characteristic(HAVStepsChar(bus, 1, self))
        self.add_characteristic(HAVPresChar(bus, 2, self))
        self.energy_expended = 0
        
 
class HAVTempChar(Characteristic):
    #HR_MSRMT_UUID = '00002a37-0000-1000-8000-00805f9b34fb'

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
        GObject.timeout_add(5000, self.temp_measure)
        
        
    def temp_measure(self):
        global temp_data
        value = []
        value.append(dbus.Byte('T'))
        value.append(temp_data["Data"])
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value }, [])
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
        
class HAVStepsChar(Characteristic):
    #HR_MSRMT_UUID = '00002a37-0000-1000-8000-00805f9b34fb'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                STEPS_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update_steps(self):
        print('Update Steps')
        if not self.notifying:
            return
        GObject.timeout_add(5000, self.step_measure)
        
        
    def step_measure(self):
        global step_data
        value = []
        value.append(dbus.Byte('S'))
        value.append(step_data["Data"])
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update_steps()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update_steps()
        
class HAVPresChar(Characteristic):
    #HR_MSRMT_UUID = '00002a37-0000-1000-8000-00805f9b34fb'

    def __init__(self, bus, index, service):
        Characteristic.__init__(
                self, bus, index,
                PRESSURE_CHAR_UUID,
                ['notify'],
                service)
        self.notifying = False
        #self.hr_ee_count = 0
   
    def update_pressure(self):
        print('Update Pressure')
        if not self.notifying:
            return
        GObject.timeout_add(5000, self.pressure_measure)
        
        
    def pressure_measure(self):
        global pres_data
        value = []
        value.append(dbus.Byte('P'))
        value.append(pres_data["Data"])
        print('Updating value: ' + repr(value))
        self.PropertiesChanged(GATT_CHRC_IFACE, { 'Value': value }, [])
        return self.notifying	

    def StartNotify(self):
        if self.notifying:
            print('Already notifying, nothing to do')
            return

        self.notifying = True
        self.update_pressure()

    def StopNotify(self):
        if not self.notifying:
            print('Not notifying, nothing to do')
            return

        self.notifying = False
        self.update_pressure()
    

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

    mainloop = GObject.MainLoop()

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


   


