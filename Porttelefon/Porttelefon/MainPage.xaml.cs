using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using Windows.Foundation;
using Windows.UI;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media;

// Required APIs to use Bluetooth GATT
using Windows.Devices.Bluetooth;
using Windows.Devices.Bluetooth.GenericAttributeProfile;

// Required APIs to use built in GUIDs
using Windows.Devices.Enumeration;

// Required APIs for buffer manipulation & async operations
using Windows.Storage.Streams;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Runtime.InteropServices.WindowsRuntime;
using System.Text;
using Porttelefon.Bluetooth;

namespace Porttelefon
{
    // Disable warning "...execution of the current method continues before the call is completed..."
#pragma warning disable 4014

    // Disable warning to "consider using the 'await' operator to await non-blocking API calls"
#pragma warning disable 1998
    public sealed partial class MainPage : Page
    {
        const int KEYS = 6;
        const string SENSOR_GUID_PREFIX = "F000AA";

        const string BUTTONS_GUID_STR = "0000FFE0-0000-1000-8000-00805F9B34FB";
        const string SENSOR_PERIOD_GUID_SUFFFIX = "3-0451-4000-B000-000000000000";

        private GattDeviceService[] serviceList = new GattDeviceService[7];

        private DeviceWatcher deviceWatcher = null;

        private DeviceInformationDisplay DeviceInfoConnected = null;

        //Handlers for device detection
        private TypedEventHandler<DeviceWatcher, DeviceInformation> handlerAdded = null;
        private TypedEventHandler<DeviceWatcher, DeviceInformationUpdate> handlerUpdated = null;
        private TypedEventHandler<DeviceWatcher, DeviceInformationUpdate> handlerRemoved = null;
        private TypedEventHandler<DeviceWatcher, Object> handlerEnumCompleted = null;

        private DeviceWatcher blewatcher = null;
        private TypedEventHandler<DeviceWatcher, DeviceInformation> OnBLEAdded = null;
        private TypedEventHandler<DeviceWatcher, DeviceInformationUpdate> OnBLEUpdated = null;
        private TypedEventHandler<DeviceWatcher, DeviceInformationUpdate> OnBLERemoved = null;

        TaskCompletionSource<string> providePinTaskSrc;
        TaskCompletionSource<bool> confirmPinTaskSrc;

        private enum MessageType { YesNoMessage, OKMessage };
        public ObservableCollection<DeviceInformationDisplay> ResultCollection
        {
            get;
            private set;
        }

        public MainPage()
        {
            this.InitializeComponent();
            txtCurrentState.Text = "IDLE";
            ResultCollection = new ObservableCollection<DeviceInformationDisplay>();
        }

        ~MainPage()
        {
            //StopWatcher();
        }


        private void btnSearchBLE_Click(object sender, RoutedEventArgs e)
        {
            StartWatcher();
        }

        //Watcher for Bluetooth LE Devices based on the Protocol ID
        private void StartWatcher()
        {
            string aqsFilter;

            ResultCollection.Clear();

            // Request the IsPaired property so we can display the paired status in the UI
            string[] requestedProperties = { "System.Devices.Aep.IsPaired" };

            //for bluetooth LE Devices
            aqsFilter = "System.Devices.Aep.ProtocolId:=\"{bb7bb05e-5972-42b5-94fc-76eaa7084d49}\"";

            deviceWatcher = DeviceInformation.CreateWatcher(
                aqsFilter,
                requestedProperties,
                DeviceInformationKind.AssociationEndpoint
                );

            // Hook up handlers for the watcher events before starting the watcher

            handlerAdded = async (watcher, deviceInfo) =>
            {
                // Since we have the collection databound to a UI element, we need to update the collection on the UI thread.
                this.Dispatcher.RunAsync(CoreDispatcherPriority.Low, () =>
                {
                    Debug.WriteLine("Watcher Add: " + deviceInfo.Id);
                    ResultCollection.Add(new DeviceInformationDisplay(deviceInfo));
                    //UpdatePairingButtons();
                });
            };
            deviceWatcher.Added += handlerAdded;

            handlerUpdated = async (watcher, deviceInfoUpdate) =>
            {
                // Since we have the collection databound to a UI element, we need to update the collection on the UI thread.
                Dispatcher.RunAsync(CoreDispatcherPriority.Low, () =>
                {
                    Debug.WriteLine("Watcher Update: " + deviceInfoUpdate.Id);
                    // Find the corresponding updated DeviceInformation in the collection and pass the update object
                    // to the Update method of the existing DeviceInformation. This automatically updates the object
                    // for us.
                    foreach (DeviceInformationDisplay deviceInfoDisp in ResultCollection)
                    {
                        if (deviceInfoDisp.Id == deviceInfoUpdate.Id)
                        {
                            deviceInfoDisp.Update(deviceInfoUpdate);
                            break;
                        }
                    }
                });
            };
            deviceWatcher.Updated += handlerUpdated;



            handlerRemoved = async (watcher, deviceInfoUpdate) =>
            {
                // Since we have the collection databound to a UI element, we need to update the collection on the UI thread.
                Dispatcher.RunAsync(CoreDispatcherPriority.Low, () =>
                {
                    Debug.WriteLine("Watcher Remove: " + deviceInfoUpdate.Id);
                    // Find the corresponding DeviceInformation in the collection and remove it
                    foreach (DeviceInformationDisplay deviceInfoDisp in ResultCollection)
                    {
                        if (deviceInfoDisp.Id == deviceInfoUpdate.Id)
                        {
                            ResultCollection.Remove(deviceInfoDisp);
                            //UpdatePairingButtons();
                            if (ResultCollection.Count == 0)
                            {
                                txtCurrentState.Text = "Searching for Bluetooth LE Devices...";
                            }
                            break;
                        }
                    }
                });
            };
            deviceWatcher.Removed += handlerRemoved;

            handlerEnumCompleted = async (watcher, obj) =>
            {
                Dispatcher.RunAsync(CoreDispatcherPriority.Low, () =>
                {
                    Debug.WriteLine($"Found {ResultCollection.Count} Bluetooth LE Devices");

                    if (ResultCollection.Count > 0)
                    {
                        txtCurrentState.Text = "Select a device for pairing";
                        for (int i = 0; i < ResultCollection.Count; ++i)
                        {
                            var TextBlock = new TextBlock();
                            TextBlock.Text = "Name: " + ResultCollection[i].Name + "Id: " + ResultCollection[i].Id;
                            SearchScroll.Content = TextBlock;
                        }
                    }
                    else
                    {
                        txtCurrentState.Text = "No Bluetooth LE Devices found.";
                    }
                    //UpdatePairingButtons();
                });
            };

            deviceWatcher.EnumerationCompleted += handlerEnumCompleted;

            deviceWatcher.Start();
        }

        // ---------------------------------------------------
        //             Pairing Process Handlers and Functions -- Begin
        // ---------------------------------------------------

        private void ShowPairingPanel(string text, DevicePairingKinds pairingKind)
        {
            pairingPanel.Visibility = Visibility.Collapsed;
            pinEntryTextBox.Visibility = Visibility.Collapsed;
            okButton.Visibility = Visibility.Collapsed;
            yesButton.Visibility = Visibility.Collapsed;
            noButton.Visibility = Visibility.Collapsed;
            pairingTextBlock.Text = text;

            switch (pairingKind)
            {
                case DevicePairingKinds.ConfirmOnly:
                case DevicePairingKinds.DisplayPin:
                    // Don't need any buttons
                    break;
                case DevicePairingKinds.ProvidePin:
                    pinEntryTextBox.Text = "";
                    pinEntryTextBox.Visibility = Visibility.Visible;
                    okButton.Visibility = Visibility.Visible;
                    break;
                case DevicePairingKinds.ConfirmPinMatch:
                    yesButton.Visibility = Visibility.Visible;
                    noButton.Visibility = Visibility.Visible;
                    break;
            }

            pairingPanel.Visibility = Visibility.Visible;
        }

        private void HidePairingPanel()
        {
            pairingPanel.Visibility = Visibility.Collapsed;
            pairingTextBlock.Text = "";
        }

        // If pin is not provided, then any pending pairing request is abandoned.
        private void CompleteProvidePinTask(string pin = null)
        {
            if (providePinTaskSrc != null)
            {
                providePinTaskSrc.SetResult(pin);
                providePinTaskSrc = null;
            }
        }


        // If pin is not provided, then any pending pairing request is abandoned.
        private void CompleteConfirmPinTask(bool accept)
        {
            if (confirmPinTaskSrc != null)
            {
                confirmPinTaskSrc.SetResult(accept);
                confirmPinTaskSrc = null;
            }
        }

        private void okButton_Click(object sender, RoutedEventArgs e)
        {
            // OK button is only used for the ProvidePin scenario
            CompleteProvidePinTask(pinEntryTextBox.Text);
            HidePairingPanel();
        }

        private void yesButton_Click(object sender, RoutedEventArgs e)
        {
            CompleteConfirmPinTask(true);
            HidePairingPanel();
        }

        private void noButton_Click(object sender, RoutedEventArgs e)
        {
            CompleteConfirmPinTask(false);
            HidePairingPanel();
        }

        private async Task<string> GetPinFromUserAsync()
        {
            HidePairingPanel();
            CompleteProvidePinTask(); // Abandon any previous pin request.

            ShowPairingPanel(
                "Please enter the PIN shown on the device you're pairing with",
                DevicePairingKinds.ProvidePin);

            providePinTaskSrc = new TaskCompletionSource<string>();

            return await providePinTaskSrc.Task;
        }

        private async Task<bool> GetUserConfirmationAsync(string pin)
        {
            HidePairingPanel();
            CompleteConfirmPinTask(false); // Abandon any previous request.

            ShowPairingPanel(
                "Does the following PIN match the one shown on the device you are pairing?: " + pin,
                DevicePairingKinds.ConfirmPinMatch);

            confirmPinTaskSrc = new TaskCompletionSource<bool>();

            return await confirmPinTaskSrc.Task;
        }

        private async void PairingRequestedHandler(
             DeviceInformationCustomPairing sender,
             DevicePairingRequestedEventArgs args)
        {
            switch (args.PairingKind)
            {
                case DevicePairingKinds.ConfirmOnly:
                    // Windows itself will pop the confirmation dialog as part of "consent" if this is running on Desktop or Mobile
                    // If this is an App for 'Windows IoT Core' where there is no Windows Consent UX, you may want to provide your own confirmation.
                    args.Accept();
                    break;

                case DevicePairingKinds.DisplayPin:
                    // We just show the PIN on this side. The ceremony is actually completed when the user enters the PIN
                    // on the target device. We automatically except here since we can't really "cancel" the operation
                    // from this side.
                    args.Accept();

                    // No need for a deferral since we don't need any decision from the user
                    await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
                    {
                        ShowPairingPanel(
                            "Please enter this PIN on the device you are pairing with: " + args.Pin,
                            args.PairingKind);

                    });
                    break;

                case DevicePairingKinds.ProvidePin:
                    // A PIN may be shown on the target device and the user needs to enter the matching PIN on 
                    // this Windows device. Get a deferral so we can perform the async request to the user.
                    var collectPinDeferral = args.GetDeferral();

                    await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
                    {
                        string pin = await GetPinFromUserAsync();
                        if (!string.IsNullOrEmpty(pin))
                        {
                            args.Accept(pin);
                        }

                        collectPinDeferral.Complete();
                    });
                    break;

                case DevicePairingKinds.ConfirmPinMatch:
                    // We show the PIN here and the user responds with whether the PIN matches what they see
                    // on the target device. Response comes back and we set it on the PinComparePairingRequestedData
                    // then complete the deferral.
                    var displayMessageDeferral = args.GetDeferral();

                    await Dispatcher.RunAsync(CoreDispatcherPriority.Normal, async () =>
                    {
                        bool accept = await GetUserConfirmationAsync(args.Pin);
                        if (accept)
                        {
                            args.Accept();
                        }

                        displayMessageDeferral.Complete();
                    });
                    break;
            }
        }

        //Watcher for Bluetooth LE Services
        private void StartBLEWatcher()
        {
            int discoveredServices = 0;
            // Hook up handlers for the watcher events before starting the watcher
            OnBLEAdded = async (watcher, deviceInfo) =>
            {
                Dispatcher.RunAsync(CoreDispatcherPriority.Low, async () =>
                {
                    Debug.WriteLine("OnBLEAdded: " + deviceInfo.Id);
                    GattDeviceService service = await GattDeviceService.FromIdAsync(deviceInfo.Id);
                    if (service != null)
                    {
                        int sensorIdx = -1;
                        string svcGuid = service.Uuid.ToString().ToUpper();
                        Debug.WriteLine("Found Service: " + svcGuid);

                        // Add this service to the list if it conforms to the TI-GUID pattern for most sensors
                        if (svcGuid.StartsWith(SENSOR_GUID_PREFIX))
                        {
                            // The character at this position indicates the index into the serviceList 
                            // container that we want to save this service to.  The rest of this program
                            // assumes that specific sensor types are at specific indexes in this array
                            sensorIdx = svcGuid[6] - '0';
                        }
                        // otherwise, if this is the GUID for the KEYS, then handle it special
                        else if (svcGuid == BUTTONS_GUID_STR)
                        {
                            sensorIdx = KEYS;
                        }
                        // If the index is legal and a service hasn't already been cached, then
                        // cache this service in our serviceList
                        if (((sensorIdx >= 0) && (sensorIdx <= KEYS)) && (serviceList[sensorIdx] == null))
                        {
                            serviceList[sensorIdx] = service;
                            //await enableSensor(sensorIdx);
                            System.Threading.Interlocked.Increment(ref discoveredServices);
                        }

                        // When all sensors have been discovered, notify the user

                        //SensorList.IsEnabled = true;
                        DisableButton.IsEnabled = true;
                        EnableButton.IsEnabled = true;
                        discoveredServices = 0;
                        txtCurrentState.Text = "Sensors on!";

                    }
                });
            };
        }

        private async void WriteData()
        {
            GattDeviceService gattService = serviceList[0];
            var characteristicList = gattService.GetCharacteristics(new Guid(SENSOR_GUID_PREFIX + 0 + SENSOR_PERIOD_GUID_SUFFFIX));
            GattCharacteristic characteristic = characteristicList[0];
            var writer = new Windows.Storage.Streams.DataWriter();
            writer.WriteByte((Byte)0x02);
            await characteristic.WriteValueAsync(writer.DetachBuffer());
        }

        private void btnSendData_Click(object sender, RoutedEventArgs e)
        {
            WriteData();
        }

        private async void pairButton_Click_1(object sender, RoutedEventArgs e)
        {
            //DeviceInformationDisplay deviceInfoDisp = listSearchResult.SelectedItem as DeviceInformationDisplay;
            DeviceInformationDisplay deviceInfoDisp = ResultCollection[0];
            txtCurrentState.Text = "Waiting for pairing";
            if (deviceInfoDisp != null)
            {
                bool paired = true;
                if (deviceInfoDisp.IsPaired != true)
                {
                    paired = false;
                    DevicePairingKinds ceremoniesSelected = DevicePairingKinds.ConfirmOnly | DevicePairingKinds.DisplayPin | DevicePairingKinds.ProvidePin | DevicePairingKinds.ConfirmPinMatch;
                    DevicePairingProtectionLevel protectionLevel = DevicePairingProtectionLevel.Default;

                    // Specify custom pairing with all ceremony types and protection level EncryptionAndAuthentication
                    DeviceInformationCustomPairing customPairing = deviceInfoDisp.DeviceInformation.Pairing.Custom;

                    customPairing.PairingRequested += PairingRequestedHandler;
                    DevicePairingResult result = await customPairing.PairAsync(ceremoniesSelected, protectionLevel);

                    customPairing.PairingRequested -= PairingRequestedHandler;

                    if (result.Status == DevicePairingResultStatus.Paired)
                    {
                        paired = true;
                    }
                    else
                    {
                        txtCurrentState.Text = "Pairing Failed " + result.Status.ToString();
                    }
                }

                if (paired)
                {
                    // device is paired, set up the sensor Tag            
                    txtCurrentState.Text = "Pairing succesful";

                    DeviceInfoConnected = deviceInfoDisp;

                    //Start watcher for Bluetooth LE Services
                    StartBLEWatcher();
                }

            }
        }

        // ---------------------------------------------------
        //             Pairing Process Handlers and Functions -- End
        // ---------------------------------------------------



    }
}
