using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace IoTCore.Time
{
    class TimeKeeper
    {
        public async void getTime() {
            var date = await Task.FromResult<string>(DateTime.Now.ToString());
            Debug.WriteLine(date);
        }
    }
}
