using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Bluetooth;

using Windows.Devices.Bluetooth.Rfcomm;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;
using Windows.Devices.Enumeration;

using System.Diagnostics;
namespace Porttelefon
{
    class BluetoothStream
    {
        private DeviceInformationCollection deviceCollection;
        private DeviceInformation selectedDevice;
        public RfcommDeviceService deviceService;
        private string deviceName = "HC-06"; /* Device name can be found at pairing page in raspberry */
        private StreamSocket streamSocket = new StreamSocket();

        public BluetoothStream()
        {
            //this.InitialiceBluetooth();
        }

        public async void InitialiceBluetooth()
        {
            string device1 = RfcommDeviceService.GetDeviceSelector(RfcommServiceId.SerialPort);
            if (device1.Count() < 1)
            {
                Debug.WriteLine("No Devices");
            }

            try
            {
                deviceCollection = await Windows.Devices.Enumeration.DeviceInformation.FindAllAsync(device1);
            }
            catch (Exception exe)
            {
                Debug.WriteLine(exe.Message);
            }

            ConnectToDevice();
        }

        private async void ConnectToDevice()
        {
            foreach(var item in deviceCollection)
            {
                if (item.Name == deviceName)
                {
                    selectedDevice = item;
                    break;
                }
            }
            if (selectedDevice == null)
            {
                Debug.WriteLine("Cannot find the device specified; Please check the device name");
                return;
            }
            else
            {
                deviceService = await RfcommDeviceService.FromIdAsync(selectedDevice.Id);

                if (deviceService != null)
                {
                    try
                    {
                        await streamSocket.ConnectAsync(deviceService.ConnectionHostName, deviceService.ConnectionServiceName);
                    }
                    catch(Exception exe)
                    {
                        Debug.WriteLine(exe.Message);
                    }

                }
                else
                {
                    Debug.WriteLine("Didn't find the specified bluetooth device");
                }
            }
        }


        public async void OpenDoor()
        {
            if (deviceService != null)
            {
                try
                {
                    string sendData = "Open door (Command)";

                    DataWriter dwriter = new DataWriter(streamSocket.OutputStream);
                    UInt32 len = dwriter.MeasureString(sendData);
                    dwriter.WriteUInt32(len);
                    dwriter.WriteString(sendData);
                    await dwriter.StoreAsync();
                    await dwriter.FlushAsync();
                }
                catch(Exception exe)
                {
                    Debug.WriteLine(exe.Message);
                }
            }
            else
            {
                Debug.WriteLine("Bluetooth is not connected correctl!");
            }

        }
    }
}
