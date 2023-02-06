/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.emergencyResponse");
var listenerAlert;
var listenerBSM;

const UNAVAILABLE_SPEED = 8191;
const MSTOMPH = 2.23694;
//enumeration values for siren_in_use. The enum comes from J2735 ASN1 standard fpr CARMA.
const siren_in_use = {
    UNAVAILABLE: 0,
    NOT_IN_USE: 1,
    IN_USE: 2,
    RESERVED: 3
}


// enumeration values for lightbar_in_use. The enum comes from J2735 ASN1 standard fpr CARMA.
const lightbar_in_use = {
    UNAVAILABLE: 0,
    NOT_IN_USE: 1,
    IN_USE: 2,
    YELLOW_CAUTION_LIGHTS: 3,
    SCHOOL_BUS_LIGHTS: 4,
    ARROW_SIGNS_ACTIVE: 5,
    SLOW_MOVING_VEHICLE: 6,
    FREQ_STOPS: 7
}

//Display vehicle information
var subscribe_bsm = () => {
    listenerBSM = new ROSLIB.Topic({
        ros: ros,
        name: '/bsm_outbound',
        messageType: 'cav_msgs/BSM'
    });

    listenerBSM.subscribe(function (message) {
        if (message.core_data != undefined && message.core_data.latitude != undefined && message.core_data.longitude != undefined) {
            $("#positionValue").text(message.core_data.latitude + "," + message.core_data.longitude);
        } else {
            $("#positionValue").text("NA");
        }

        if (message.core_data != undefined && message.core_data.speed != undefined) {
            if (message.core_data.speed != UNAVAILABLE_SPEED) {
                let speedMPH = Math.round(message.core_data.speed * MSTOMPH);
                $("#velocityValue").text(speedMPH + " MPH");
            }
        } else {
            $("#speedMPH").text("NA");
        }

        if (message.part_ii != undefined && message.part_ii.length > 0 && message.part_ii[0].special_vehicle_extensions != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_alerts != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use != undefined) {
            if (message.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use["siren_in_use"] == siren_in_use.IN_USE) {
                $("#sirenValue").text("ON");
            } else if (message.part_ii[0].special_vehicle_extensions.vehicle_alerts.siren_use["siren_in_use"] == siren_in_use.NOT_IN_USE) {
                $("#sirenValue").text("OFF");
            } else {
                $("#sirenValue").text("NA");
            }
        } else {
            $("#sirenValue").text("NA");
        }

        if (message.part_ii != undefined && message.part_ii.length > 0 && message.part_ii[0].special_vehicle_extensions != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_alerts != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use != undefined) {
            if (message.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use["lightbar_in_use"] == lightbar_in_use.IN_USE) {
                $("#lightValue").text("ON");
            } else if (message.part_ii[0].special_vehicle_extensions.vehicle_alerts.lights_use["lightbar_in_use"] == lightbar_in_use.NOT_IN_USE) {
                $("#lightValue").text("OFF");
            } else {
                $("#lightValue").text("NA");
            }
        } else {
            $("#lightValue").text("NA");
        }
    });
}

//Display alert and play sound
var subscribe_alert = () => {
    listenerAlert = new ROSLIB.Topic({
        ros: ros,
        name: '/emergency_vehicle_ui_warning',
        messageType: 'cav_msgs/UIInstructions'
    });
    listenerAlert.subscribe(function (message) {
        if (message != undefined && message.msg != undefined) {
            if ($("#emergencyAlert").length != 0) {
                document.getElementById("emergencyAlertMsg").innerHTML = "<strong>" + message.msg + "</strong>";
                $("#emergencyAlert").addClass("show");
                document.getElementById('audioAlert3').play();
            }

        }
    });
}

//Service call to get route name
var service_get_emergency_route = () => {
    var get_emergency_route = new ROSLIB.Service({
        ros: ros,
        name: '/get_emergency_route',
        serviceType: 'cav_srvs/GetEmergencyRoute'
    });

    var request = new ROSLIB.ServiceRequest({
    });

    get_emergency_route.callService(request, function (result) {
        $("#routeNameValue").text(result.route_name);
    });
}

//Service call arrive at emergency destination
var service_arrive_at_emergency_destination = () => {
    let success = false;
    var arrive_at_emergency_destination = new ROSLIB.Service({
        ros: ros,
        name: '/arrived_at_emergency_destination',
        serviceType: 'std_srvs/Trigger.srv'
    });

    var request = new ROSLIB.ServiceRequest({
    });

    arrive_at_emergency_destination.callService(request, function (result) {
        success = result.success;
    });
    if (!success) {
        alert("Service call failed to remove emergency route from BSM.");
    }
    return success;
}

//Reset alert and sounds every 5 secs
setInterval(() => {
    if ($("#emergencyAlert").length == 0) {
        let alert = createAlertDiv("");
        document.getElementById("alert-col").appendChild(alert);
    }
    $("#emergencyAlert").removeClass("show");
    document.getElementById('audioAlert3').pause();
}, 5000);

//Create alert element
var createAlertDiv = (message) => {
    let alert = document.createElement('div');
    alert.className = "alert alert-danger alert-dismissible fade";
    alert.id = "emergencyAlert";
    let alertTxt = document.createElement("span");
    alertTxt.innerHTML = "<string>" + message + "</strong>";
    alertTxt.id = "emergencyAlertMsg";
    alert.appendChild(alertTxt);
    let alertCloseBtn = document.createElement('button');
    alertCloseBtn.className = "close";
    alertCloseBtn.setAttribute("data-dismiss", "alert");
    alertCloseBtn.setAttribute("aria-label", "Close");
    let alertSpan = document.createElement('span');
    alertSpan.setAttribute("aria-hidden", "true");
    alertSpan.innerHTML = "&times;";
    alertCloseBtn.appendChild(alertSpan);
    alert.appendChild(alertCloseBtn);
    return alert;
}

var goToEventManagement = () => {
    $("#emergencyAlert").remove();
    if (listenerAlert != undefined) {
        listenerAlert.unsubscribe();
    }
    if (listenerBSM != undefined) {
        listenerBSM.unsubscribe();
    }

    document.getElementById('audioAlert3').pause();
    $('#divCarmaMessengerView').css('display', '');
    $('#divWidgetArea').css('display', 'none');
    $('#divWidgetAreaEventManagement').css('display', '');
    $('#divWidgetAreaEmergencyResponse').css('display', 'none');
    $('#Messenger_back_arrow').css('display', 'inline-block');
    $('#divCarmaMessengerMenu').css('display', 'none');

    //show event management widget
    CarmaJS.WidgetFramework.closeEventManagementWidgets();
    CarmaJS.WidgetFramework.loadEventManagementWidgets();
}

//Load wiget on startup
CarmaJS.WidgetFramework.emergencyResponse = (function () {
    $.widget("CarmaJS.emergencyResponse", {
        _create: function () {
            let container = document.createElement('div');
            container.className = "emergency-response-container";
            //create title div
            let titleRow = document.createElement('div');
            titleRow.className = "row title-row";
            let titleCol = document.createElement('div');
            titleCol.className = "col";
            let titleImg = "../images/siren.png";
            let img = document.createElement('img');
            img.src = titleImg;
            img.className = "title-img";
            titleCol.appendChild(img);
            let titleLbl = document.createElement('Label');
            titleLbl.innerHTML = "Emergency Response Vehicle Information";
            titleLbl.className = "titleLbl";
            titleCol.appendChild(titleLbl);
            titleRow.appendChild(titleCol);

            /**
             * create Vehicle status div: bsm_id, position, velocity, siren status, light status
             */
            let vehicleStatusRow = document.createElement('div');
            vehicleStatusRow.className = "row vehicle-status-row";

            //position
            let positionCol = document.createElement('div');
            positionCol.className = "col"
            var positionLabel = document.createElement('Label');
            positionLabel.innerHTML = "Position";
            positionLabel.className = "position-lbl";
            var positionValue = document.createElement('Label');
            positionValue.innerHTML = "NA";
            positionValue.id = "positionValue";
            positionCol.appendChild(positionLabel);
            positionCol.appendChild(positionValue);
            vehicleStatusRow.appendChild(positionCol);

            //velocity
            let velocityCol = document.createElement('div');
            velocityCol.className = "col"
            var velocityLabel = document.createElement('Label');
            velocityLabel.innerHTML = "Velocity";
            velocityLabel.className = "velocity-lbl";
            var velocityValue = document.createElement('Label');
            velocityValue.innerHTML = "NA";
            velocityValue.id = "velocityValue";
            velocityCol.appendChild(velocityLabel);
            velocityCol.appendChild(velocityValue);
            vehicleStatusRow.appendChild(velocityCol);

            //siren status
            let sirenCol = document.createElement('div');
            sirenCol.className = "col"
            var sirenLabel = document.createElement('Label');
            sirenLabel.innerHTML = "Siren Status";
            sirenLabel.className = "siren-lbl";
            var sirenValue = document.createElement('Label');
            sirenValue.innerHTML = "NA";
            sirenValue.id = "sirenValue";
            sirenCol.appendChild(sirenLabel);
            sirenCol.appendChild(sirenValue);
            vehicleStatusRow.appendChild(sirenCol);

            //light status
            let lightCol = document.createElement('div');
            lightCol.className = "col";
            var lightLabel = document.createElement('Label');
            lightLabel.innerHTML = "Light Status";
            lightLabel.className = "light-lbl";
            var lightValue = document.createElement('Label');
            lightValue.innerHTML = "NA";
            lightValue.id = "lightValue";
            lightCol.appendChild(lightLabel);
            lightCol.appendChild(lightValue);
            vehicleStatusRow.appendChild(lightCol);

            /**
             * Route selction div and arrive at destination button
             */
            let destinationRow = document.createElement('div');
            destinationRow.className = "row destination-row";
            let destinationCol = document.createElement('div');
            destinationCol.className = "col col-4";
            var destLbl = document.createElement('Label');
            destLbl.innerHTML = "Emergency Destination";
            destLbl.className = "dest-lbl";
            destinationCol.appendChild(destLbl);
            let destTbl = document.createElement('table');
            destTbl.className = "table";
            let destHeader = document.createElement('thead');
            let destTr = document.createElement('tr');
            let destThRouteName = document.createElement('th');
            destThRouteName.textContent = "Route Name";
            destTr.appendChild(destThRouteName);
            destHeader.appendChild(destTr);
            destTbl.appendChild(destHeader);
            let destBody = document.createElement('tbody');
            let destTr1 = document.createElement('tr');
            let destTrRouteName = document.createElement('th');
            destTrRouteName.textContent = "NA";
            destTrRouteName.id = "routeNameValue";
            destTr1.appendChild(destTrRouteName);
            destBody.appendChild(destTr1);
            destTbl.appendChild(destBody);
            destinationCol.appendChild(destTbl);
            //Destination Button
            let destBtn = document.createElement('button');
            destBtn.className = "btn btn-danger dest-btn btn-lg";
            destBtn.innerHTML = "Arrived at Emergency Location";
            destBtn.setAttribute("title", "You will be redirected to event management page.");
            destBtn.onclick = () => {
                let success = service_arrive_at_emergency_destination();
                if(success)
                {
                    goToEventManagement();
                }                
            };
            destinationCol.appendChild(destBtn);
            destinationRow.appendChild(destinationCol);

            //Alert popup 
            let alertRow = document.createElement('div');
            alertRow.className = "row alert-row";
            let alertCol = document.createElement('div');
            alertCol.className = "col offset-sm-7";
            alertCol.id = "alert-col";
            let alert = createAlertDiv("");
            alertCol.appendChild(alert);
            alertRow.appendChild(alertCol);
            container.appendChild(titleRow);
            container.appendChild(vehicleStatusRow);
            container.appendChild(destinationRow);
            container.appendChild(alertRow);
            $(this.element).append(container);
        },
        subscribe_bsm: function () {
            subscribe_bsm();
        },
        service_get_emergency_route: function () {
            service_get_emergency_route();
        },
        subscribe_alert: function () {
            subscribe_alert();
        },
        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("Emergency Response configuration page is destroyed");
        }
    });//CarmaJS.

    var loadCustomWidget = function (container) {
        //create Emergency Response page
        container.emergencyResponse();
        container.emergencyResponse("subscribe_bsm", null);
        container.emergencyResponse("subscribe_alert", null);
        container.emergencyResponse("service_get_emergency_route", null);
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();