/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.emergencyResponse");
var listenerAlert;
var listenerBSM;
var map = null;
const ERV_ROUTE_SOURCE = "erv-route-trace";
var data = {
    'type': 'FeatureCollection',
    'features': [
        {
            'type': 'Feature',
            'geometry': {
                'type': 'Point',
                'coordinates': []
            }
        }
    ]
}
var destination_pin = null;


const UNAVAILABLE_SPEED = 8191;
const MSTOMPH = 2.23694;
//enumeration values for siren_in_use. The enum comes from J2735 ASN1 standard for CARMA.
const siren_in_use = {
    UNAVAILABLE: 0,
    NOT_IN_USE: 1,
    IN_USE: 2,
    RESERVED: 3
}


// enumeration values for lightbar_in_use. The enum comes from J2735 ASN1 standard for CARMA.
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
            //Vehicle current location
            $("#positionValue").text(message.core_data.latitude + "," + message.core_data.longitude);
            data.features = [];
            let feature = createFeature();
            feature.geometry.coordinates.push(message.core_data.longitude, message.core_data.latitude);
            data.features.push(feature);

            //Vehicle route
            if (message.regional != undefined && message.regional.length > 0 && message.regional[0].route_destination_points != undefined
                && message.regional[0].route_destination_points.length > 0) {
                message.regional[0].route_destination_points.forEach(element => {
                    let route_feature = createFeature();
                    route_feature.geometry.coordinates.push(element.longitude, element.latitude);
                    data.features.push(route_feature);
                });
            }
            map.getSource(ERV_ROUTE_SOURCE).setData(data);
            if (data.features.length > 1) {
                if (destination_pin == null) {
                    destination_pin = createMarker(data.features[data.features.length - 1].geometry.coordinates);
                }
                destination_pin.setLngLat(data.features[data.features.length - 1].geometry.coordinates);
            }
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
var createFeature = () => {
    let feature = {
        'type': 'Feature',
        'geometry': {
            'type': 'Point',
            'coordinates': []
        }
    };
    return feature;
}

var createMarker = ([longitude, latitude]) => {
    let marker = new mapboxgl.Marker({
        color: "#FF0000"
    }).setLngLat([latitude, longitude]).addTo(map);
    return marker;
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
        if (result.is_successful) {
            $("#routeNameValue").text(result.route_name);
        }
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
/***
 * Load OpenStreetMap with MapboxGL JS
 */
var loadMap = () => {
    mapboxgl.accessToken = "pk.eyJ1IjoiZGR1MjAyMCIsImEiOiJjbDJyeHJob2YwYnhwM2xtaG9zaDdnYTR4In0.Rh2bSS44c99BoDj2W7jjfw";
    let default_center = [-77.150495, 38.955675];
    map = new mapboxgl.Map({
        container: 'erv-map',
        style: 'mapbox://styles/mapbox/satellite-v9',
        center: default_center,
        zoom: 17
    });
    map.addControl(new mapboxgl.FullscreenControl());

    setInterval(() => {
        //Change View Point
        map.jumpTo({ 'center': data.features[0].geometry.coordinates.length == 0 ? default_center : data.features[0].geometry.coordinates, 'zoom': 17 });
    }, 2000);

    map.on('load', () => {
        map.addSource(ERV_ROUTE_SOURCE, {
            'type': 'geojson',
            'data': data
        });

        map.addLayer({
            'id': ERV_ROUTE_SOURCE,
            'type': 'circle',
            'source': 'erv-route-trace',
            'paint': {
                'circle-radius': 6,
                'circle-color': '#3bb2d0'
            },
            'filter': ['==', '$type', 'Point']
        });

        //Add destination pin
        if (data.features.length > 1) {
            destination_pin = createMarker(data.features[data.features.length - 1].geometry.coordinates);
        }
    });
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
            destinationCol.className = "col";
            //Destination Button
            let destBtn = document.createElement('button');
            destBtn.className = "btn btn-danger dest-btn btn-lg";
            destBtn.innerHTML = "Arrived at Emergency Location";
            destBtn.setAttribute("title", "You will be redirected to event management page.");
            destBtn.onclick = () => {
                let success = service_arrive_at_emergency_destination();
                if (success) {
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

            //MAP 
            let mapRow = document.createElement('div');
            mapRow.className = "row map-row flex-grow-1";
            let mapCol = document.createElement('div');
            mapCol.className = "col map-col";
            mapCol.id = "erv-map";
            mapRow.appendChild(mapCol);

            container.appendChild(titleRow);
            container.appendChild(vehicleStatusRow);
            container.appendChild(destinationRow);
            container.appendChild(alertRow);
            container.appendChild(mapRow);
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
        loadMap: function () {
            loadMap();
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
        container.emergencyResponse("loadMap", null);
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();