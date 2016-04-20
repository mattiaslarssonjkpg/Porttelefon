using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Data.Json;

namespace IoTCore.Httprequest
{
    class ParJSON
    {
        private const String idKey = "ID";

        /**
         * FUNCTION NAME : ToJsonObject
         * DESCRIPTION   : 
         * INPUT         : -
         * OUTPUT        : - 
         * NOTE          : -
         */
        public JsonObject ToJsonObject(String value) {
            JsonObject DeviceJson = new JsonObject();
            DeviceJson.SetNamedValue(idKey, JsonValue.CreateStringValue(value);

            return DeviceJson;
        }
    } 
}
