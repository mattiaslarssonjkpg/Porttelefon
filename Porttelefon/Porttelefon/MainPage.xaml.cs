﻿using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Windows.Foundation;
using Windows.UI;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;

// Required APIs to use built in GUIDs
using Windows.Devices.Enumeration;

// Required APIs for buffer manipulation & async operations
using Windows.Storage.Streams;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;

using System.Threading;
namespace Porttelefon
{
    // Disable warning "...execution of the current method continues before the call is completed..."
#pragma warning disable 4014

    // Disable warning to "consider using the 'await' operator to await non-blocking API calls"
#pragma warning disable 1998
    public sealed partial class MainPage : Page
    {
        StartupTask _server = new StartupTask();

        public MainPage()
        {
            this.InitializeComponent();
            Task.Run(_server.Run);
        }

        ~MainPage()
        {

        }

    }
}