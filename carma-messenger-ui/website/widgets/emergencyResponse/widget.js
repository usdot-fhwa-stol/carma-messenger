/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.emergencyResponse");
var listenerAlert;
var listenerBSM;
//Initialize map object
var erv_map = null;
const ERV_ROUTE_SOURCE = "erv-route-trace";
//Initialize dataset
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

// Emergency vehicle class enumeration based on J2735 BasicVehicleClass
const emergency_vehicle_classes = {
    60: "Unknown Emergency Vehicle",
    61: "Other Emergency Vehicle (Federal)",
    62: "Fire Light Vehicle",
    63: "Fire Heavy Vehicle",
    64: "Fire Paramedic Vehicle",
    65: "Fire Ambulance Vehicle",
    66: "Police Light Vehicle",
    67: "Police Heavy Vehicle",
    68: "Other Emergency Responder",
    69: "Other Ambulance Vehicle"
}

const basic_vehicle_roles = {
    0: "Basic Vehicle",
    1: "Public Transport",
    2: "Special Transport",
    3: "Dangerous Goods",
    4: "Road Work",
    5: "Road Rescue",
    6: "Emergency",
    7: "Safety Car",
    8: "None/Unknown",
    9: "Truck",
    10: "Motorcycle",
    11: "Road Side Source",
    12: "Police",
    13: "Fire",
    14: "Ambulance",
    15: "DOT",
    16: "Transit",
    17: "Slow Moving",
    18: "Stop NGO",
    19: "Cyclist",
    20: "Pedestrian",
    21: "Non-Motorized",
    22: "Military"
};

var current_vehicle_class = 60; // Default to unknown
var current_vehicle_role = 8; // Default to NONE_UNKNOWN
var erv_plugin_enabled = false; // Track current plugin status

//Display vehicle information
var subscribe_bsm = () => {
    listenerBSM = new ROSLIB.Topic({
        ros: ros,
        name: '/bsm_outbound',
        messageType: 'carma_v2x_msgs/msg/BSM'
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
            erv_map.getSource(ERV_ROUTE_SOURCE).setData(data);
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

        // Display current vehicle class from BSM
        if (message.part_ii != undefined && message.part_ii.length > 0 && message.part_ii[0].special_vehicle_extensions != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_classification != undefined
            && message.part_ii[0].special_vehicle_extensions.vehicle_classification.basic_vehicle_class != undefined) {
            let vehicle_class = message.part_ii[0].special_vehicle_extensions.vehicle_classification.basic_vehicle_class;
            current_vehicle_class = vehicle_class;
            $("#vehicleClassValue").text(emergency_vehicle_classes[vehicle_class] || "Unknown");
            $("#vehicleClassSelect").val(vehicle_class);
        } else {
            $("#vehicleClassValue").text("Not Set");
        }

        // Display current vehicle role from BSM
        if (message.part_ii != undefined && message.part_ii.length > 0 && message.part_ii[0].supplemental_vehicle_extensions != undefined
            && message.part_ii[0].supplemental_vehicle_extensions.class_details != undefined
            && message.part_ii[0].supplemental_vehicle_extensions.class_details.role != undefined
            && message.part_ii[0].supplemental_vehicle_extensions.class_details.role.basic_vehicle_role != undefined) {
            let vehicle_role = message.part_ii[0].supplemental_vehicle_extensions.class_details.role.basic_vehicle_role;
            current_vehicle_role = vehicle_role;
            $("#vehicleRoleValue").text(basic_vehicle_roles[vehicle_role] || "Unknown");
            $("#vehicleRoleSelect").val(vehicle_role);
        } else {
            $("#vehicleRoleValue").text("Not Set");
        }
    });
}

//Service call to update emergency vehicle class using ROS service instead of parameters
var service_update_vehicle_class = (vehicle_class) => {
    var setParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        parameters: [{
            name: 'emergency_vehicle_class',
            value: {
                type: 2, // PARAMETER_INTEGER
                integer_value: parseInt(vehicle_class)
            }
        }]
    });

    setParamService.callService(request, function(result) {
        if (result && result.results && result.results.length > 0 && result.results[0].successful) {
            console.log("Vehicle class parameter updated successfully to:", vehicle_class);
            current_vehicle_class = parseInt(vehicle_class);
            $("#vehicleClassValue").text(emergency_vehicle_classes[vehicle_class] || "Unknown");
        } else {
            console.error("Failed to update vehicle class parameter:", result);
            alert("Failed to update vehicle class parameter");
            // Revert dropdown to previous value
            $("#vehicleClassSelect").val(current_vehicle_class);
        }
    });
}

//Service call to update emergency vehicle role using ROS service
var service_update_vehicle_role = (vehicle_role) => {
    var setParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        parameters: [{
            name: 'emergency_vehicle_role',
            value: {
                type: 2, // PARAMETER_INTEGER
                integer_value: parseInt(vehicle_role)
            }
        }]
    });

    setParamService.callService(request, function(result) {
        if (result && result.results && result.results.length > 0 && result.results[0].successful) {
            console.log("Vehicle role parameter updated successfully to:", vehicle_role);
            current_vehicle_role = parseInt(vehicle_role);
            $("#vehicleRoleValue").text(basic_vehicle_roles[vehicle_role] || "Unknown");
        } else {
            console.error("Failed to update vehicle role parameter:", result);
            alert("Failed to update vehicle role parameter");
            $("#vehicleRoleSelect").val(current_vehicle_role);
        }
    });
}

//Service call to toggle ERV plugin enable/disable using ROS service instead of parameters
var toggle_erv_plugin_parameter = (enable) => {
    var setParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/set_parameters',
        serviceType: 'rcl_interfaces/srv/SetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        parameters: [{
            name: 'enable_emergency_response_vehicle_plugin',
            value: {
                type: 1, // PARAMETER_BOOL
                bool_value: enable
            }
        }]
    });

    setParamService.callService(request, function(result) {
        if (result && result.results && result.results.length > 0 && result.results[0].successful) {
            erv_plugin_enabled = enable;
            updateERVPluginButtonState();
            console.log("ERV plugin parameter", enable ? "enabled" : "disabled");

            // Show status message
            let statusMsg = enable ? "ERV Plugin Activated - BSM publishing started" : "ERV Plugin Deactivated - BSM publishing stopped";
            showStatusMessage(statusMsg, enable ? "success" : "warning");
        } else {
            console.error("Failed to set ERV plugin parameter:", result);
            alert("Failed to set ERV plugin parameter");
            // Revert button state
            updateERVPluginButtonState();
        }
    });
}

var get_erv_plugin_status = () => {
    var getParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/get_parameters',
        serviceType: 'rcl_interfaces/srv/GetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        names: ['enable_emergency_response_vehicle_plugin']
    });

    getParamService.callService(request, function(result) {
        if (result && result.values && result.values.length > 0) {
            let paramValue = result.values[0];
            if (paramValue.type === 1) { // PARAMETER_BOOL
                erv_plugin_enabled = paramValue.bool_value;
                updateERVPluginButtonState();
                console.log("Current ERV plugin status:", paramValue.bool_value);
            } else {
                console.warn("Unexpected parameter type for ERV plugin status:", paramValue.type);
                erv_plugin_enabled = false;
                updateERVPluginButtonState();
            }
        } else {
            console.warn("Failed to get ERV plugin status parameter or empty result");
            // Set default state
            erv_plugin_enabled = false;
            updateERVPluginButtonState();
        }
    });
}

//Get current vehicle class using ROS service
var get_vehicle_class = () => {
    var getParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/get_parameters',
        serviceType: 'rcl_interfaces/srv/GetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        names: ['emergency_vehicle_class']
    });

    getParamService.callService(request, function(result) {
        if (result && result.values && result.values.length > 0) {
            let paramValue = result.values[0];
            if (paramValue.type === 2) { // PARAMETER_INTEGER
                current_vehicle_class = paramValue.integer_value;
                $("#vehicleClassValue").text(emergency_vehicle_classes[current_vehicle_class] || "Unknown");
                $("#vehicleClassSelect").val(current_vehicle_class);
                console.log("Current vehicle class:", current_vehicle_class);
            } else {
                console.warn("Unexpected parameter type for vehicle class:", paramValue.type);
            }
        } else {
            console.warn("Failed to get vehicle class parameter or empty result");
        }
    });
}

//Get current vehicle role using ROS service
var get_vehicle_role = () => {
    var getParamService = new ROSLIB.Service({
        ros: ros,
        name: '/emergency_response_vehicle_plugin_node/get_parameters',
        serviceType: 'rcl_interfaces/srv/GetParameters'
    });

    var request = new ROSLIB.ServiceRequest({
        names: ['emergency_vehicle_role']
    });

    getParamService.callService(request, function(result) {
        if (result && result.values && result.values.length > 0) {
            let paramValue = result.values[0];
            if (paramValue.type === 2) { // PARAMETER_INTEGER
                current_vehicle_role = paramValue.integer_value;
                $("#vehicleRoleValue").text(basic_vehicle_roles[current_vehicle_role] || "Unknown");
                $("#vehicleRoleSelect").val(current_vehicle_role);
                console.log("Current vehicle role:", current_vehicle_role);
            } else {
                console.warn("Unexpected parameter type for vehicle role:", paramValue.type);
            }
        } else {
            console.warn("Failed to get vehicle role parameter or empty result");
        }
    });
}

//Enhanced parameter monitoring using service calls (polling approach)
var monitor_parameters_with_services = () => {
    // Poll parameters every 5 seconds to detect changes
    setInterval(() => {
        get_erv_plugin_status();
        get_vehicle_class();
        get_vehicle_role();
    }, 5000);
}

//Alternative: Enhanced parameter events subscription with better error handling
var subscribe_to_parameter_events = () => {
    var param_events_listener = new ROSLIB.Topic({
        ros: ros,
        name: '/parameter_events',
        messageType: 'rcl_interfaces/msg/ParameterEvent'
    });

    param_events_listener.subscribe(function(message) {
        // Check if this event is for our node and parameter
        if (message.node === '/emergency_response_vehicle_plugin_node') {
            // Handle new parameters
            if (message.new_parameters && message.new_parameters.length > 0) {
                message.new_parameters.forEach(function(param) {
                    if (param.name === 'enable_emergency_response_vehicle_plugin' && param.value.type === 1) {
                        erv_plugin_enabled = param.value.bool_value;
                        updateERVPluginButtonState();
                        console.log("ERV plugin parameter added/changed to:", param.value.bool_value);
                    }
                    if (param.name === 'emergency_vehicle_class' && param.value.type === 2) {
                        current_vehicle_class = param.value.integer_value;
                        $("#vehicleClassValue").text(emergency_vehicle_classes[current_vehicle_class] || "Unknown");
                        $("#vehicleClassSelect").val(current_vehicle_class);
                        console.log("Vehicle class parameter added/changed to:", current_vehicle_class);
                    }
                });
            }

            // Handle changed parameters
            if (message.changed_parameters && message.changed_parameters.length > 0) {
                message.changed_parameters.forEach(function(param) {
                    if (param.name === 'enable_emergency_response_vehicle_plugin' && param.value.type === 1) {
                        erv_plugin_enabled = param.value.bool_value;
                        updateERVPluginButtonState();
                        console.log("ERV plugin parameter changed to:", param.value.bool_value);
                    }
                    if (param.name === 'emergency_vehicle_class' && param.value.type === 2) {
                        current_vehicle_class = param.value.integer_value;
                        $("#vehicleClassValue").text(emergency_vehicle_classes[current_vehicle_class] || "Unknown");
                        $("#vehicleClassSelect").val(current_vehicle_class);
                        console.log("Vehicle class parameter changed to:", current_vehicle_class);
                    }
                });
            }
        }
    });
}

//Update ERV plugin button state and text
var updateERVPluginButtonState = () => {
    let button = $("#ervPluginToggle");
    if (erv_plugin_enabled) {
        button.removeClass("btn-success").addClass("btn-danger");
        button.text("Disable ERV Plugin");
        button.attr("title", "Click to stop BSM publishing and deactivate ERV functionality");
        $("#ervStatusValue").text("ACTIVE").removeClass("text-danger").addClass("text-success");
    } else {
        button.removeClass("btn-danger").addClass("btn-success");
        button.text("Enable ERV Plugin");
        button.attr("title", "Click to start BSM publishing and activate ERV functionality");
        $("#ervStatusValue").text("INACTIVE").removeClass("text-success").addClass("text-danger");
    }
}

//Show temporary status message
var showStatusMessage = (message, type) => {
    // Remove existing status message if any
    $(".status-message").remove();

    // Create status message element
    let statusDiv = document.createElement('div');
    statusDiv.className = `alert alert-${type === 'success' ? 'success' : 'warning'} alert-dismissible fade show status-message`;
    statusDiv.style.position = 'fixed';
    statusDiv.style.top = '20px';
    statusDiv.style.right = '20px';
    statusDiv.style.zIndex = '9999';
    statusDiv.style.minWidth = '300px';

    statusDiv.innerHTML = `
        <strong>${message}</strong>
        <button type="button" class="close" data-dismiss="alert" aria-label="Close">
            <span aria-hidden="true">&times;</span>
        </button>
    `;

    document.body.appendChild(statusDiv);

    // Auto-remove after 5 seconds
    setTimeout(() => {
        $(statusDiv).fadeOut(500, function() {
            $(this).remove();
        });
    }, 5000);
}

//create feature point with empty coordinates
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

//create Marker with initialized geolocation and add marker to the map
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
        messageType: 'carma_msgs/msg/UIInstructions'
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

//Service call arrive at emergency destination
var service_arrive_at_emergency_destination = () => {
    let success = false;
    var arrive_at_emergency_destination = new ROSLIB.Service({
        ros: ros,
        name: '/arrived_at_emergency_destination',
        serviceType: 'std_srvs/srv/Trigger.srv'
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
}, 10000);

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
    //Default view is at TFHRC
    let default_center = [-77.150495, 38.955675];
    erv_map = new mapboxgl.Map({
        container: 'erv-map',
        style: 'mapbox://styles/mapbox/satellite-v9',
        center: default_center,
        zoom: 17
    });
    erv_map.addControl(new mapboxgl.FullscreenControl());

    setInterval(() => {
        //Change View Point
        erv_map.jumpTo({ 'center': data.features[0].geometry.coordinates.length == 0 ? default_center : data.features[0].geometry.coordinates, 'zoom': 17 });
    }, 2000);

    erv_map.on('load', () => {
        erv_map.addSource(ERV_ROUTE_SOURCE, {
            'type': 'geojson',
            'data': data
        });

        erv_map.addLayer({
            'id': ERV_ROUTE_SOURCE,
            'type': 'circle',
            'source': 'erv-route-trace',
            'paint': {
                'circle-radius': 10,
                'circle-color': '#FFAC1C'
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
             * create Vehicle status div: bsm_id, position, velocity, siren status, light status, vehicle class
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
             * ERV Plugin Control Row
             */
            let pluginControlRow = document.createElement('div');
            pluginControlRow.className = "row plugin-control-row";

            // Current plugin status display
            let statusCol = document.createElement('div');
            statusCol.className = "col-md-6";
            var statusLabel = document.createElement('Label');
            statusLabel.innerHTML = "Plugin Status";
            statusLabel.className = "erv-status-lbl";
            statusLabel.style.display = "block"; // Add this line
            var statusValue = document.createElement('Label');
            statusValue.innerHTML = "INACTIVE";
            statusValue.id = "ervStatusValue";
            statusValue.className = "erv-status-value text-danger";
            statusValue.style.display = "block"; // Add this line
            statusCol.appendChild(statusLabel);
            statusCol.appendChild(statusValue);

            // Plugin toggle button
            let toggleCol = document.createElement('div');
            toggleCol.className = "col-md-6";
            var toggleLabel = document.createElement('Label');
            toggleLabel.innerHTML = "ERV Plugin Control";
            toggleLabel.className = "erv-toggle-lbl";
            toggleLabel.style.display = "block"; // Add this line
            var toggleButton = document.createElement('button');
            toggleButton.className = "btn btn-success erv-toggle-btn";
            toggleButton.id = "ervPluginToggle";
            toggleButton.innerHTML = "Enable ERV Plugin";
            toggleButton.setAttribute("title", "Click to start BSM publishing and activate ERV functionality");
            toggleButton.style.marginTop = "5px"; // Add this line

            toggleButton.onclick = function() {
                let newState = !erv_plugin_enabled;
                toggle_erv_plugin_parameter(newState);
            };

            toggleCol.appendChild(toggleLabel);
            toggleCol.appendChild(toggleButton);
            pluginControlRow.appendChild(toggleCol);
            pluginControlRow.appendChild(statusCol);

            /**
             * Vehicle Class Selection Row
             */
            let vehicleClassRow = document.createElement('div');
            vehicleClassRow.className = "row vehicle-class-row";

            // Current vehicle class display
            let currentClassCol = document.createElement('div');
            currentClassCol.className = "col-md-6";
            var currentClassLabel = document.createElement('Label');
            currentClassLabel.innerHTML = "Current Vehicle Class";
            currentClassLabel.className = "vehicle-class-lbl";
            var currentClassValue = document.createElement('Label');
            currentClassValue.innerHTML = "Not Set";
            currentClassValue.id = "vehicleClassValue";
            currentClassValue.className = "vehicle-class-value";
            currentClassCol.appendChild(currentClassLabel);
            currentClassCol.appendChild(currentClassValue);
            vehicleClassRow.appendChild(currentClassCol);

            // Vehicle class selection dropdown
            let classSelectCol = document.createElement('div');
            classSelectCol.className = "col-md-6";
            var classSelectLabel = document.createElement('Label');
            classSelectLabel.innerHTML = "Select Vehicle Class";
            classSelectLabel.className = "vehicle-class-select-lbl";
            classSelectLabel.setAttribute("for", "vehicleClassSelect");
            var classSelect = document.createElement('select');
            classSelect.className = "form-control vehicle-class-select";
            classSelect.id = "vehicleClassSelect";

            // Populate dropdown options
            Object.keys(emergency_vehicle_classes).forEach(key => {
                let option = document.createElement('option');
                option.value = key;
                option.text = `${key} - ${emergency_vehicle_classes[key]}`;
                classSelect.appendChild(option);
            });

            classSelect.onchange = function() {
                let selectedClass = this.value;
                if (selectedClass && selectedClass != current_vehicle_class) {
                    service_update_vehicle_class(selectedClass);
                }
            };

            classSelectCol.appendChild(classSelectLabel);
            classSelectCol.appendChild(classSelect);
            vehicleClassRow.appendChild(classSelectCol);

            /**
             * Vehicle Role Selection Row
             */
            let vehicleRoleRow = document.createElement('div');
            vehicleRoleRow.className = "row vehicle-role-row";

            // Current vehicle role display
            let currentRoleCol = document.createElement('div');
            currentRoleCol.className = "col-md-6";
            var currentRoleLabel = document.createElement('Label');
            currentRoleLabel.innerHTML = "Current Vehicle Role";
            currentRoleLabel.className = "vehicle-role-lbl";
            var currentRoleValue = document.createElement('Label');
            currentRoleValue.innerHTML = "Not Set";
            currentRoleValue.id = "vehicleRoleValue";
            currentRoleValue.className = "vehicle-role-value";
            currentRoleCol.appendChild(currentRoleLabel);
            currentRoleCol.appendChild(currentRoleValue);
            vehicleRoleRow.appendChild(currentRoleCol);

            // Vehicle role selection dropdown
            let roleSelectCol = document.createElement('div');
            roleSelectCol.className = "col-md-6";
            var roleSelectLabel = document.createElement('Label');
            roleSelectLabel.innerHTML = "Select Vehicle Role";
            roleSelectLabel.className = "vehicle-role-select-lbl";
            roleSelectLabel.setAttribute("for", "vehicleRoleSelect");
            var roleSelect = document.createElement('select');
            roleSelect.className = "form-control vehicle-role-select";
            roleSelect.id = "vehicleRoleSelect";

            // Populate dropdown options
            Object.keys(basic_vehicle_roles).forEach(key => {
                let option = document.createElement('option');
                option.value = key;
                option.text = `${key} - ${basic_vehicle_roles[key]}`;
                roleSelect.appendChild(option);
            });

            roleSelect.onchange = function() {
                let selectedRole = this.value;
                if (selectedRole && selectedRole != current_vehicle_role) {
                    service_update_vehicle_role(selectedRole);
                }
            };

            roleSelectCol.appendChild(roleSelectLabel);
            roleSelectCol.appendChild(roleSelect);
            vehicleRoleRow.appendChild(roleSelectCol);

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
            container.appendChild(pluginControlRow); // Add the plugin control row
            container.appendChild(vehicleClassRow); // Add the vehicle class row
            container.appendChild(vehicleRoleRow); // Add the vehicle role row
            container.appendChild(destinationRow);
            container.appendChild(alertRow);
            container.appendChild(mapRow);
            $(this.element).append(container);
        },
        subscribe_bsm: function () {
            subscribe_bsm();
        },
        subscribe_alert: function () {
            subscribe_alert();
        },
        loadMap: function () {
            loadMap();
        },
        getERVStatus: function () {
            get_erv_plugin_status();
        },
        getVehicleClass: function () {
            get_vehicle_class();
        },
        subscribeParameterEvents: function () {
            subscribe_to_parameter_events();
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
        container.emergencyResponse("loadMap", null);
        container.emergencyResponse("getERVStatus", null); // Get initial ERV status
        container.emergencyResponse("getVehicleClass", null);
        container.emergencyResponse("subscribeParameterEvents", null); // Monitor parameter changes
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget
    };
})();
