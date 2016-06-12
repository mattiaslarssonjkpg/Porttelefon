using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Http;
using Windows.ApplicationModel.Background;
using Windows.System.Threading;
using System.Threading.Tasks;

namespace AgilaProject.Socket
{
    public class StartupTask
    {
        private SocketServer socket;

        public StartupTask() { }

        ~StartupTask() { }

        public async Task Run()
        {
            socket = new SocketServer(9000);
            await ThreadPool.RunAsync(x => {
                socket.OnError += socket_OnError;
                socket.OnDataRecived += Socket_OnDataRecived;
                socket.Star();
            });
        }

        private void Socket_OnDataRecived(string data)
        {
            if (data == "OpenDoor")
            {
                socket.Send(data + "recived, opening door");
            }
            else
            {
                socket.Send("Text Recive:" + data);
            }
        }

        private void socket_OnError(string message)
        {
            
        }
    }
}
