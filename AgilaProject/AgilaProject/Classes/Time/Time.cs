using System;
using System.Threading.Tasks;

namespace AgilaProject.Time
{
    public class KingTime
    {
        public static async Task<String> getTime()
        {
            return await Task.FromResult<string>(DateTime.Now.ToString());
        }
    }
}
