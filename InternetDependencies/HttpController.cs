using System;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Diagnostics;
using System.Net.Http;
using System.Net.Http.Headers;
using Windows.Data.Json;
using System.Collections.Generic;

namespace IoTCore.Httprequest
{
    class HttpController {

        /**
         * FUNCTION NAME : ToJsonObject
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        async Task<string> GetToken(string text, string serverAddress)
        {
            var httpClient = new HttpClient();

            var parameters = new Dictionary<string, string>();
            parameters["text"] = text;
            Task<HttpResponseMessage> response = httpClient.PostAsync(serverAddress, new FormUrlEncodedContent(parameters));

            return await response.Result.Content.ReadAsStringAsync();
        }

        /**
         * FUNCTION NAME : ToJsonObject
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        public async void PostToServer(JsonObject jsonObj, String apiKey, String serverAddress) {
            var myClient = new HttpClient();

            var content = new StringContent(jsonObj.ToString());

            myClient.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", apiKey);

            content.Headers.ContentType = new MediaTypeHeaderValue("application/json");

            HttpResponseMessage response = await myClient.PostAsync(serverAddress, content);

            // This tells if you if the client-server communication is actually using HTTP/2
            Debug.WriteLine(response.Content.ReadAsStringAsync());

        }

        public async void GetFromServer(JsonObject jsonObj, String apiKey, String serverAddress) {
            var myClient = new HttpClient();

            StringContent queryString = new StringContent(_data);
            //myClient.PostAsync(_serveraddress, _data);
            HttpResponseMessage response = await myClient.GetAsync(_serveraddress);
            
            // This tells if you if the client-server communication is actually using HTTP/2
            Debug.WriteLine(response.Content.ReadAsStringAsync());

        }
    }
}
