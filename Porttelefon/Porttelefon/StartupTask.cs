using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Http;
using Windows.ApplicationModel.Background;
using Windows.System.Threading;
using System.Threading.Tasks;

namespace Porttelefon
{
    public class StartupTask
    {
        private SocketServer socket;
        BluetoothStream MyBlueTooth = new BluetoothStream();


        public StartupTask() { }

        ~StartupTask() { }

        public async Task Run()
        {
            MyBlueTooth.InitialiceBluetooth();
            socket = new SocketServer(9000);
            await ThreadPool.RunAsync(x => {
                socket.OnError += socket_OnError;
                socket.OnDataRecived += Socket_OnDataRecived;
                socket.Star();
            });
        }

        private void Socket_OnDataRecived(string data)
        {
            if (data == "OpenDoor Tha Door")
                MyBlueTooth.OpenDoor();
            socket.Send("Text Recive:" + data);
        }

        private void socket_OnError(string message)
        {

        }
    }
}