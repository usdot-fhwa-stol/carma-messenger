
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
        //Up Track
        set upTrack(newUpTrack){
            if (newUpTrack==null || newUpTrack=="undefined")
                newUpTrack = "";
            console.log("set up track");
            sessionStorage.setItem("upTrack", newUpTrack);
        },
        get upTrack(){
            console.log("get up track");
             return sessionStorage.getItem("upTrack");
        },

        //Down Track
        set downTrack(newDownTrack){
            if (newDownTrack==null || newDownTrack=="undefined")
                newDownTrack = "";
            console.log("set down track");
            sessionStorage.setItem("downTrack", newDownTrack);
        },
        get downTrack(){
            console.log("get down track");
             return sessionStorage.getItem("downTrack");
        },

        //Minimum Gap
        set minGap(newMinGap){
            if (newMinGap==null || newMinGap=="undefined")
                newMinGap = "";
            console.log("set minimum gap");
            sessionStorage.setItem("minGap", newMinGap);
        },
        get minGap(){
            console.log("get minimum gap");
             return sessionStorage.getItem("minGap");
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