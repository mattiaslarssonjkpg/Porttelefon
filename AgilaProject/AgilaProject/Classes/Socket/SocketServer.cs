using System;
using System.Diagnostics;
using Windows.Networking.Sockets;
using Windows.Storage.Streams;

namespace AgilaProject.Socket
{
    internal class SocketServer
    {
        private readonly int _port;
        public int Port { get { return _port; } }

        private StreamSocketListener listener;
        private DataWriter _writer;

        public delegate void DataRecived(string data);
        public event DataRecived OnDataRecived;

        public delegate void Error(string message);
        public event Error OnError;

        public SocketServer(int port)
        {
            _port = port;
        }

        public async void Star()
        {
            try
            {
                if (listener != null)
                {
                    await listener.CancelIOAsync();
                    listener.Dispose();
                    listener = null;
                }


                listener = new StreamSocketListener();

                listener.ConnectionReceived += Listener_ConnectionReceived;
                await listener.BindServiceNameAsync(Port.ToString());
            }
            catch (Exception e)
            {
                if (OnError != null)
                    OnError(e.Message);
            }
        }

        private async void Listener_ConnectionReceived(StreamSocketListener sender, StreamSocketListenerConnectionReceivedEventArgs args)
        {
            var reader = new DataReader(args.Socket.InputStream);
            _writer = new DataWriter(args.Socket.OutputStream);
            try
            {
                while (true)
                {
                    uint sizeFieldCount = await reader.LoadAsync(sizeof(uint));
                    if (sizeFieldCount != sizeof(uint))
                        return;

                    uint stringLenght = reader.ReadUInt32();
                    uint actualStringLength = await reader.LoadAsync(stringLenght);
                    if (stringLenght != actualStringLength)
                        return;
                    if (OnDataRecived != null)
                    {
                        string data = reader.ReadString(actualStringLength);
                        OnDataRecived(data);
                    }
                }

            }
            catch (Exception ex)
            {
                if (OnError != null)
                    OnError(ex.Message);
            }
        }

        public async void Send(string message)
        {
            if (_writer != null)
            {
                _writer.WriteUInt32(_writer.MeasureString(message));
                _writer.WriteString(message);

                try
                {
                    await _writer.StoreAsync();
                    await _writer.FlushAsync();
                }
                catch (Exception ex)
                {
                    if (OnError != null)
                        OnError(ex.Message);
                }
            }
        }
    }
}
