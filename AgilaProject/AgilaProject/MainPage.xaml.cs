using System;
using System.Collections.Generic;
using Windows.ApplicationModel.Core;
using Windows.UI.Core;
using Windows.UI.ViewManagement;
using Windows.UI.Xaml;
using Windows.UI.Xaml.Controls;
using System.Threading.Tasks;
using System.Diagnostics;

// Start ************** My includes ************
using AgilaProject.Models;
using static AgilaProject.InternetDependencies;
using AgilaProject.InternetDependencies;
using AgilaProject.RFID;
using AgilaProject.Time;
using AgilaProject.Socket;

// Stop  ************** My includes ************

namespace AgilaProject
{
    public sealed partial class MainPage : Page
    {
        //private List<Teacher> ListViewTeachers;
        internal List<Teacher> TeachersList;
        internal String 
            httpGetAllTeachers = "http://193.10.30.154/DeveloperAPI/api/Users/1",
            httpGetSchedule = "http://193.10.30.154/DeveloperAPI",
            httpPostMail = "http://localhost:17877/api/mail/sendmail";


        //internal static HttpController httpController;
        internal HttpController httpController;
        StartupTask _server = new StartupTask();
        public MainPage()
        {
            this.InitializeComponent();
            Task.Run(_server.Run);
            httpController = new HttpController();

            //IToastText01 trying_toast = ToastContentFactory.CreateToastText01();
            //trying_toast.TextBodyWrap.Text = "Toast";
            //ScheduledToastNotification giveTime = new ScheduledToastNotification(trying_toast.GetXml(), DateTime.Now.AddSeconds(2));
            //giveTime.Id = "test";
            //ToastNotificationManager.CreateToastNotifier().AddToSchedule(giveTime);
            test();
            runTask();
        }
        private async void test()
        {
            httpPostMail = "http://localhost:5998/api/Users/Mail/" + "4";
            RegisterBindingModel registermodel = new RegisterBindingModel();
            registermodel.StudentName = "Test";
            registermodel.Subject = "Corridor Notification";
            registermodel.Content = "is outside and wish you to open the door";
            registermodel.Id = "4";
            String JsonObj = httpController.ConvertToJson(registermodel);
            String x = await httpController.PostToServer(JsonObj, httpPostMail);
            Debug.WriteLine(x);
            int y = 0;
        }
        private void runTask()
        {
            Task t = Task.Run(async() =>
            {
                while (true)
                {
                    String response = await getTeachers();
                    TeachersList = TeacherManager.ConvertFromJson(response);
                    
                    await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal,
                    () => {
                        ListViewPopulate.Items.Clear();
                        foreach (var element in TeachersList)
                        {
                            ListViewPopulate.Items.Add(element);                
                        }
                    });

                    Debug.Write("I was here");
                    await Task.Delay(TimeSpan.FromSeconds(5));
                }
            });
        }
        //Collect new Value when student scan thier Student Card
        //We need an eventhandler who listen on the change's on the IRQ pin
        
        internal async Task<String> getTeachers()
        {
            String respone = await httpController.GetFromServer(httpGetAllTeachers);
            return respone;
        }

        private async void GridView_ItemClick(object sender, ItemClickEventArgs e)
        {
            var Teacher = (Teacher)e.ClickedItem;
            CoreApplicationView newView = CoreApplication.CreateNewView();
            int newViewId = 0;
            await newView.Dispatcher.RunAsync(CoreDispatcherPriority.Normal, () =>
            {
                Frame frame = new Frame();
                frame.Navigate(typeof(TeacherPage), Teacher);
                Window.Current.Content = frame;
                // You have to activate the window in order to show it later.
                Window.Current.Activate();

                newViewId = ApplicationView.GetForCurrentView().Id;
            });
            bool viewShown = await ApplicationViewSwitcher.TryShowAsStandaloneAsync(newViewId);
        }
    }
    #region goodGarbage
    /*ThreadPoolTimer timer1 = ThreadPoolTimer.CreatePeriodicTimer(async (t) =>
        {
            
            try { 
                String x = await test();
                
            }
            catch (Exception ex)
            {
                Debug.WriteLine(ex);
            }
        }, TimeSpan.FromSeconds(10));

        internal async void timerCallback(object state) {
            // do some work not connected with UI
            Debug.WriteLine("i was here");
            
            await CoreApplication.MainView.CoreWindow.Dispatcher.RunAsync(CoreDispatcherPriority.Normal,
            () => {
                foreach (var element in TeacherManager.getTeachers()){
                    ListViewPopulate.Items.Add(element);
                }
            });

        }*/
    #endregion
}
