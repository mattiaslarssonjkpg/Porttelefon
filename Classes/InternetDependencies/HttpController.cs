using System;
using System.Threading.Tasks;
using System.Net.Http;
using System.Net.Http.Headers;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Net;

namespace AgilaProject.InternetDependencies
{
    public class HttpController {
        public class RegisterBindingModel {
            public String Id { get; set; }
            public String TeacherEmail { get; set; }
            public String Subject { get; set; }
            public String StudentName { get; set; }
            public String Content { get; set; }
        }

        /**
         * FUNCTION NAME : ToJsonObject
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        public String ConvertToJson(RegisterBindingModel content)
        {
            var person = new RegisterBindingModel { Id = content.Id , TeacherEmail = content.TeacherEmail, Content = content.Content, StudentName = content.StudentName};
            var JsonObj = JsonConvert.SerializeObject(person);
            return JsonObj;
        }

        /**
         * FUNCTION NAME : ConvertFromJson
         * DESCRIPTION   : Converts JsonString to regular String
         * INPUT         : String
         * OUTPUT        : String 
         * NOTE          : -
         */
        public String ConvertFromJson(String value)
        {
            var Lines = JsonConvert.DeserializeObject<RegisterBindingModel>(value);
            return Lines.Id;
        }


        /**
         * FUNCTION NAME : ToJsonObject
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        public async Task<string> GetToken(string text, string serverAddress)
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
        public async Task<string> PostToServer(String jsonObj, String serverAddress)
        {
            var myClient = new HttpClient();
            var content = new StringContent(jsonObj);

            //myClient.DefaultRequestHeaders.Authorization = new AuthenticationHeaderValue("Bearer", apiKey);
            content.Headers.ContentType = new MediaTypeHeaderValue("application/json");

            HttpResponseMessage response = await myClient.PostAsync(serverAddress, content);
            // This tells if you if the client-server communication is actually using HTTP/2
            if(response.StatusCode == HttpStatusCode.OK)
                return await response.Content.ReadAsStringAsync();
            else return null;

        }

        internal async Task<string> GetFromServer(String serverAddress)
        {
            var myClient = new HttpClient();

            HttpResponseMessage response = await myClient.GetAsync(serverAddress);

            if (response.StatusCode == HttpStatusCode.OK)
                return await response.Content.ReadAsStringAsync();
            else return null;
            // This tells if you if the client-server communication is actually using HTTP/2
        }
    }
}
