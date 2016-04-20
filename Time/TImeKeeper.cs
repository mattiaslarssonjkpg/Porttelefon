using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IoTCore.Time
{
    class TimeKeeper
    {
        protected async void getTime()
        {
            // Task.FromResult is a placeholder for actual work that returns a string.
            var today = await Task.FromResult<string>(DateTime.Now.DayOfWeek.ToString());
        }
    }
}
