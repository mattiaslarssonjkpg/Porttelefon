using AgilaProject.InternetDependencies;
using AgilaProject.Models;
using AgilaProject.RFID;
using System;
using System.Diagnostics;
using System.Text;
using System.Threading.Tasks;
using Windows.ApplicationModel.Core;
using Windows.UI.Core;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using Windows.UI.Xaml.Media.Imaging;
using Windows.UI.Xaml.Navigation;
using static AgilaProject.InternetDependencies.HttpController;

namespace AgilaProject
{
    public sealed partial class TeacherPage : Page
    {
        private String httpPostMail = "http://193.10.30.154/DeveloperAPI/api/Users/{uID}/Mail"; //ändra med rätt adress
        internal static CoreApplicationView coreApplikationView;
        internal MFRC522 RFIDReader;
        internal Teacher TeacherAttributs;
        internal HttpController httpController;
        internal String 
            MACAddress = null, 
            Teacherstatus = "avaiable",
            JsonObj = null;
        internal Boolean isInit = false;
        public TeacherPage()
        {
            this.InitializeComponent();
            httpController = new HttpController();
            RFIDReader = new MFRC522();
            InitRFIDReader();
            runTask();
        }
        private async void InitRFIDReader()
        {
            if (!isInit) { 
                await RFIDReader.openMFRC522();
                RFIDReader.PCD_SetAntennaGain(0x70);
                isInit = true;
            }
        }
        protected override void OnNavigatedTo(NavigationEventArgs e)
        {
            base.OnNavigatedTo(e);
            TeacherAttributs = (Teacher)e.Parameter;
            TeacherImage.Source = new BitmapImage(new Uri(this.BaseUri, TeacherAttributs.TeacherImage));
            TeacherName.Text = TeacherAttributs.Name;
            TeacherStatus.Text = TeacherAttributs.Status;
            Debug.WriteLine(TeacherAttributs.Email);
            Debug.WriteLine(TeacherAttributs.Id);
            InstructionsText.Text = "Please, scan your Hj Card";
            coreApplikationView = CoreApplication.GetCurrentView();
            StackPanelMail.Visibility = Visibility.Collapsed;
            Notify_Teacher.Visibility = Visibility.Collapsed;
            StackPanelIsMailSent.Visibility = Visibility.Collapsed;
        }

        private void runTask()
        {
            Task t = Task.Run(async () =>
            {
                Boolean isHjCardScan = false;
                while (true)
                {
                    if(!isHjCardScan)
                    { 
                        if (isInit) //since init is a async task we must wait before we can run RFID functions
                        {
                            if (RFIDReader.PICC_IsNewCardPresent())
                            {
                                if (RFIDReader.PICC_ReadCardSerial())
                                {
                                    MACAddress = BitConverter.ToString(RFIDReader.uid.uidByte);
                                    await coreApplikationView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal,
                                    () => {
                                        if (TeacherAttributs.Status.ToLower() != Teacherstatus)
                                        {
                                            InstructionsText.Text = "Teacher is " +  TeacherAttributs.Status +", Follow these instructions if you want to request meeting with teacher";
                                            StackPanelMail.Visibility = Visibility.Visible;
                                            Notify_Teacher.Visibility = Visibility.Collapsed;
                                            JuCard.Text = MACAddress; //symoblize Student name
                                            isHjCardScan = true;
                                        }
                                        else
                                        {
                                            InstructionsText.Text = "Teacher is avaiable push the button to notify teacher you're outsude";
                                            StackPanelMail.Visibility = Visibility.Collapsed;
                                            Notify_Teacher.Visibility = Visibility.Visible;
                                            isHjCardScan = true;
                                            
                                        }
                                    });
                                }
                            }
                        }
                    }
                    await Task.Delay(TimeSpan.FromMilliseconds(100));
                }
            });
        }
        
        private void Button_Click(object sender, RoutedEventArgs e)
        {
            RFIDReader._spiDevice.Dispose();
            RFIDReader._resetPin.Dispose();
            RFIDReader = null;
            Window.Current.Close();
        }

        private async void isMailSent_Click(object sender, RoutedEventArgs e)
        {
            if ((String)(sender as Button).Content.ToString().ToLower() == "close") 
            {
                Window.Current.Close();
            }
            else if((String)(sender as Button).Content.ToString().ToLower() == "send mail again?") //this means the mail not has been sent
            {
                String response = await httpController.PostToServer(JsonObj, httpPostMail);

                if (response != null)
                {
                    StackPanelIsMailSent.Visibility = Visibility.Collapsed;
                    Notify_Teacher.Visibility = Visibility.Collapsed;
                    StackPanelIsMailSent.Visibility = Visibility.Visible;
                    UserMessageIsMailSent.Text = "E-Mail has been sent!";
                    isMailSentSend.Visibility = Visibility.Collapsed;
                    isMailSentClose.Content = "Close";
                }
                else
                {
                    StackPanelMail.Visibility = Visibility.Collapsed;
                    Notify_Teacher.Visibility = Visibility.Collapsed;
                    StackPanelIsMailSent.Visibility = Visibility.Visible;
                    UserMessageIsMailSent.Text = "Failed to send the E-Mail!";
                    isMailSentClose.Content = "Close";
                    isMailSentSend.Content = "Send mail Again?";
                }
            }
        }

        private async void Book_meeting_Click(object sender, RoutedEventArgs e)
        {
            httpPostMail = "http://193.10.30.154/API/api/Users/" + TeacherAttributs.Id.ToString() + "/Mail";
            RegisterBindingModel registermodel = new RegisterBindingModel();
            registermodel.StudentName = MACAddress;
            registermodel.Subject = "Corridor Notification";
            registermodel.Content = "is outside and wish you to open the door";
            registermodel.Id = TeacherAttributs.Id.ToString();
            JsonObj = httpController.ConvertToJson(registermodel);
            String response = await httpController.PostToServer(JsonObj, httpPostMail);

            if (response != null)
            {
                StackPanelIsMailSent.Visibility = Visibility.Collapsed;
                Notify_Teacher.Visibility = Visibility.Collapsed;
                StackPanelIsMailSent.Visibility = Visibility.Visible;
                UserMessageIsMailSent.Text = "E-Mail has been sent!";
                isMailSentSend.Visibility = Visibility.Collapsed;
                isMailSentClose.Content = "Close";
            }
            else 
            {
                StackPanelMail.Visibility = Visibility.Collapsed;
                Notify_Teacher.Visibility = Visibility.Collapsed;
                StackPanelIsMailSent.Visibility = Visibility.Visible;
                UserMessageIsMailSent.Text = "Failed to send the E-Mail!";
                isMailSentClose.Content = "Close";
                isMailSentSend.Content = "Send mail Again?";
            }


        }
        
        private async void Notify_Teacher_Click(object sender, RoutedEventArgs e)
        {
            RegisterBindingModel registermodel = new RegisterBindingModel();
            registermodel.StudentName = MACAddress;
            httpPostMail = "http://193.10.30.154/DeveloperAPI/api/Users/" + TeacherAttributs.Id.ToString() + "/Mail"; 
            registermodel.Subject = MACAddress + "Meeting proposal from " + MACAddress;
            registermodel.Content = MACAddress + "Student is want to see you at " + CalenderPicker.Date.ToString() + " " + TimePicker.Time.ToString();
            registermodel.Id = TeacherAttributs.Id.ToString(); 
            JsonObj = httpController.ConvertToJson(registermodel);
            String response = await httpController.PostToServer(JsonObj, httpPostMail);
            if(response != null)
            {
                StackPanelMail.Visibility = Visibility.Collapsed;
                Notify_Teacher.Visibility = Visibility.Collapsed;
                StackPanelIsMailSent.Visibility = Visibility.Visible;
                UserMessageIsMailSent.Text = "Mail has been sent, wait a few moments to let the teacher open the door";
                isMailSentSend.Visibility = Visibility.Collapsed;
                isMailSentClose.Content = "Close";
            }
            else
            {
                StackPanelMail.Visibility = Visibility.Collapsed;
                Notify_Teacher.Visibility = Visibility.Collapsed;
                StackPanelIsMailSent.Visibility = Visibility.Visible;
                UserMessageIsMailSent.Text = "Sending the mail failed!";
                isMailSentClose.Content = "Close";
                isMailSentSend.Content = "Send mail Again?";
            }
        }
    }
}

#region goodGarabe
/*private async Task<String> sendMail()
{
    JuCard.Text = CalenderPicker.Date.ToString();
    //JuCard.Text = TimePicker.Time.ToString();
    return null;
}*/
#endregion