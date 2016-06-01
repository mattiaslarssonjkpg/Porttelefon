using Newtonsoft.Json;
using Newtonsoft.Json.Linq;
using System;
using System.Collections.Generic;

namespace AgilaProject.Models
{
    public class ScheduleModel
    {
        public int Id { get; set; }
        public List<events> events { get; set; }
    }
    public class events
    {
        public int Id { get; set; }             //"Id": 1,
        public String DTEnd { get; set; }       // "2016-05-27T16:45:00",
        public String DTStart { get; set; }     //"DTStart": "2016-05-27T13:00:00",
        public String Duration { get; set; }    //"Duration": "03:45:00",
        public String DTStamp { get; set; }     //"DTStamp": "2016-05-26T16:03:02",
        public String LastModified { get; set; }//"LastModified": "2016-01-28T11:32:29",
        public String Summary { get; set; }     //"Summary": "Agile Project - Project support - Projekt work",
        public String Location { get; set; }    //"Location": "E3303 - SystemlabbetE3317 - Reglerlabbet",
        public String externalId { get; set; }  //"externalId": "BokningsId_20160128_000000525",
        public String status { get; set; }      //"status": "Away"
    }
    public class Teacher
    {
        public int Id { get; set; }
        public String Title { get; set; }
        public String UserName { get; set; }
        public String Email { get; set; }
        public String firstName { get; set; }
        public String LastName { get; set; }
        public String Signature { get; set; }
        //public List<ScheduleModel> schedule { get; set; }
        public String Name { get; set; }
        public String Status { get; set; }
        public String TeacherImage { get; set; }
    }
    //[{"Id":1,"Title":"Tester","UserType":1,"UserName":"Test12345","Email":"test@test.com","FirstName":"Test","LastName":"testet","signature":"ScMa","schedule":null}]
    public class TeacherManager
    {

        public static List<Teacher> ConvertFromJson(String value)
        {
            var Lines = (JArray)JsonConvert.DeserializeObject(value);
            var Teachers = new List<Teacher> { };
            foreach(var b in Lines)
            {
                Teachers.Add(new Teacher {
                    Id = b.SelectToken("Id").Value<Int16>(),
                    Title = b.SelectToken("Title").Value<String>(),
                    firstName = b.SelectToken("FirstName").Value<String>(),
                    LastName = b.SelectToken("LastName").Value<String>(),
                    Name = b.SelectToken("FirstName").Value<String>() + " " + b.SelectToken("LastName").Value<String>(),
                    Status = b.SelectToken("status").Value<String>(),
                    Email = b.SelectToken("Email").Value<String>(),
                    Signature = b.SelectToken("signature").Value<String>(),
                    TeacherImage = "Assets/TeachersPic.png"
                    //schedule = b.SelectToken("schedule").Value<List<ScheduleModel>>()
                });
            }
            return Teachers;
        }
    }
}
