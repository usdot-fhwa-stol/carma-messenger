
//Load global session variables
var EventSessionFormFields = LoadSessionVariables();

$(function () {
    $('[data-toggle="tooltip"]').tooltip()
})
  
$(document).ready(function () {

     $('#card-event-management').click(function(){
       $('#divCarmaMessengerView').css('display','');
       $('#divWidgetAreaEventManagement').css('display','');
       $('#divWidgetAreaEmergencyResponse').css('display','none');
       $('#Messenger_back_arrow').css('display','inline-block');
       $('#divCarmaMessengerMenu').css('display','none');

       //show event management widget
       CarmaJS.WidgetFramework.closeEventManagementWidgets();
       CarmaJS.WidgetFramework.loadEventManagementWidgets();
    });

    $('#card-emergency-response').click(function(){
        $('#divCarmaMessengerView').css('display','');
        $('#divWidgetAreaEventManagement').css('display','none');
        $('#divWidgetAreaEmergencyResponse').css('display','');        
        $('#Messenger_back_arrow').css('display','inline-block');
        $('#divCarmaMessengerMenu').css('display','none');
 
        //show event management widget
        CarmaJS.WidgetFramework.closeEmergencyResponseWidgets();
        CarmaJS.WidgetFramework.loadEmergencyResponseWidgets();
     });

    //check whether there is any active event broadcasting
    console.log("EventSessionFormFields.BCStatus: " + EventSessionFormFields.BCStatus);
    if(EventSessionFormFields.BCStatus != null && EventSessionFormFields.BCStatus.length > 0 && EventSessionFormFields.BCStatus.toLowerCase() == "true")
    {
        $('#BCStatus').text("An event is active");
        $('#BCStatus').css({'color':'#8bc34a', 'font-weight': 'bold'});
    }else{
        $('#BCStatus').text("No event in progress");
        $('#BCStatus').css('color','black');
    }
});

function LoadSessionVariables()
{
    /*** Event management session variables
        Include: Up Track, Down Track, Minimum Gap, and Advisory Speed
    ***/
    var EventSessionFormFields = {
        // Start Latitude and Longitude
        set startLatitude(newStartLatitude){
            if (newStartLatitude==null || newStartLatitude=="undefined")
                newStartLatitude = "";
            console.log("set start latitude");
            sessionStorage.setItem("startLat", newStartLatitude);
        },
        get startLatitude(){
            console.log("get start latitude");
             return sessionStorage.getItem("startLat");
        },
        set startLongitude(newStartLongitude){
            if (newStartLongitude==null || newStartLongitude=="undefined")
                newStartLongitude = "";
            console.log("set start longitude");
            sessionStorage.setItem("startLon", newStartLongitude);
        },
        get startLongitude(){
            console.log("get start longitude");
             return sessionStorage.getItem("startLon");
        },
        // End Latitude and Longitude
        set endLatitude(newEndLatitude){
            if (newEndLatitude==null || newEndLatitude=="undefined")
                newEndLatitude = "";
            console.log("set end latitude");
            sessionStorage.setItem("endLat", newEndLatitude);
        },
        get endLatitude(){
            console.log("get end latitude");
             return sessionStorage.getItem("endLat");
        },
        set endLongitude(newEndLongitude){
            if (newEndLongitude==null || newEndLongitude=="undefined")
                newEndLongitude = "";
            console.log("set end longitude");
            sessionStorage.setItem("endLon", newEndLongitude);
        },
        get endLongitude(){
            console.log("get end longitude");
             return sessionStorage.getItem("endLon");
        },
        //Advisory Speed
        set advisorySpeed(newAdvisorySpeed){
            if (newAdvisorySpeed==null || newAdvisorySpeed=="undefined")
                newAdvisorySpeed = "";
            console.log("set advisory speed");
            sessionStorage.setItem("advisorySpeed", newAdvisorySpeed);
        },
        get advisorySpeed(){
            console.log("get advisory speed");
             return sessionStorage.getItem("advisorySpeed");
        },
        //broadcasting status
        set BCStatus(newBCStatus){
            if (newBCStatus==null || newBCStatus=="undefined")
                newBCStatus = "";
            console.log("set broadcasting status");
            sessionStorage.setItem("BCStatus", newBCStatus);
        },
        get BCStatus(){
            console.log("get broadcasting status");
            return sessionStorage.getItem("BCStatus");
        },
    }

  return EventSessionFormFields;
}