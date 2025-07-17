/***
* Create a unique namespace for each plugin widget to minimize collision of same name variables or functions.
***/
CarmaJS.registerNamespace("CarmaJS.WidgetFramework.eventManagement");

CarmaJS.WidgetFramework.eventManagement = (function () {

    //*** Private Variables ***
    var installfoldername = 'widgets/eventManagement/';
    var broadcasting_btn_status_t = {
        "STARTED": 0,
        "STOPPED": 1,
        "UNKNOWN": 2,
        "START": 3,
        "STOP": 3,
    };
    var BC_FQ_10HZ = "10HZ";
    var broadcasting_btn_status = broadcasting_btn_status_t.UNKNOWN;
    var mapInitializationAttempts = 0;
    var maxMapInitializationAttempts = 10;

    var checkSessionVariables = function() {
        if(typeof EventSessionFormFields !== 'undefined' && EventSessionFormFields != null) {
            if(EventSessionFormFields.BCStatus == "true") {
                $('#eventBroadcastBtn').text('Stop Broadcast');
                $('#eventBroadcastBtn').css('background-color','red');
                $('#BroadcastingFrequencySpan').text(BC_FQ_10HZ);
                $('#BroadcastingFrequencySpan').removeClass('hide');
                $('#BroadcastingFrequencyLabelTitle').removeClass('hide');
                $('#BroadcastingFrequencySpan').addClass('show');
                $('#BroadcastingFrequencyLabelTitle').addClass('show');
            }
            if(EventSessionFormFields.startLatitude != null) {
                $('#StartLat').val(EventSessionFormFields.startLatitude);
                $('#StartLat').prop('disabled',true);
            }
            if(EventSessionFormFields.startLongitude != null) {
                $('#StartLon').val(EventSessionFormFields.startLongitude);
                $('#StartLon').prop('disabled',true);
            }
            if(EventSessionFormFields.endLatitude != null) {
                $('#EndLat').val(EventSessionFormFields.endLatitude);
                $('#EndLat').prop('disabled',true);
            }
            if(EventSessionFormFields.endLongitude != null) {
                $('#EndLon').val(EventSessionFormFields.endLongitude);
                $('#EndLon').prop('disabled',true);
            }
            if(EventSessionFormFields.LanesBlockedLeft != null) {
                $('#LanesBlockedLeft').val(EventSessionFormFields.LanesBlockedLeft);
                $('#LanesBlockedLeft').prop('disabled',true);
            }
            if(EventSessionFormFields.LanesBlockedRight != null) {
                $('#LanesBlockedRight').val(EventSessionFormFields.LanesBlockedRight);
                $('#LanesBlockedRight').prop('disabled',true);
            }
            if(EventSessionFormFields.advisorySpeed != null) {
               $('#AdvisorySpeed').val(EventSessionFormFields.advisorySpeed);
               $('#AdvisorySpeed').prop('disabled',true);
            }
        }
    }

    var ensureMapIsVisible = function() {
        mapInitializationAttempts++;

        if (mapInitializationAttempts > maxMapInitializationAttempts) {
            console.warn('Max map initialization attempts reached');
            return;
        }

        const mapContainer = document.getElementById('load-map');
        if (!mapContainer) {
            console.log('Map container not found, creating it...');
            createMapContainer();
            setTimeout(ensureMapIsVisible, 1000);
            return;
        }

        // Check if the Event Management view is visible
        const eventView = document.getElementById('divCarmaMessengerView');
        if (!eventView || eventView.style.display === 'none') {
            console.log('Event Management view not visible, waiting...');
            setTimeout(ensureMapIsVisible, 1000);
            return;
        }

        // Check if container has dimensions
        const rect = mapContainer.getBoundingClientRect();
        if (rect.width === 0 || rect.height === 0) {
            console.log('Map container has no dimensions, waiting...');
            setTimeout(ensureMapIsVisible, 1000);
            return;
        }

        // Initialize map if not already done
        if (typeof window.initializeMapWhenReady === 'function') {
            console.log('Triggering map initialization...');
            window.initializeMapWhenReady();
        } else {
            console.log('Map initialization function not available, waiting...');
            setTimeout(ensureMapIsVisible, 1000);
        }
    }

    var createMapContainer = function() {
        // Check if we're in the Event Management area
        let targetContainer = document.getElementById('divWidgetAreaEventManagement');

        if (!targetContainer) {
            console.warn('Event Management area not found');
            return;
        }

        // Create the map container if it doesn't exist
        let mapSection = document.getElementById('map-section');
        if (!mapSection) {
            mapSection = document.createElement('div');
            mapSection.id = 'map-section';
            mapSection.className = 'col-md-6';
            mapSection.innerHTML = `
                <div class="card">
                    <div class="card-header">
                        <h5 class="card-title mb-0">Live Map View</h5>
                    </div>
                    <div class="card-body p-0">
                    <div id="map-container" style="position: relative; height: 100%;">
                        <div id="load-map" style="height: 100%;"></div>
                        <div id="marker-overlay">
                            <div class="marker-item" draggable="true" data-type="start-zone">
                                <img src="http://www.google.com/mapfiles/markerS.png" />
                            </div>
                            <div class="marker-item" draggable="true" data-type="end-zone">
                                <img src="http://www.google.com/mapfiles/markerE.png" />
                            </div>
                        </div>
                    </div>
                </div>
            `;

            // Add to the Event Management area
            targetContainer.appendChild(mapSection);
        }
    }

    var initializeEventManagementLayout = function() {
        const eventArea = document.getElementById('divWidgetAreaEventManagement');
        if (!eventArea) {
            console.warn('Event Management area not found');
            return;
        }

        // Clear the area and create a proper layout
        eventArea.innerHTML = `
            <div class="container-fluid" style="height: 100vh;">
                <div class="row" style="height: 100%;">
                    <div class="col-md-7 p-0" style="height: 100%;">
                        <div id="map-container" style="position: relative; height: 100%;">
                            <div id="load-map" style="height: 100%;"></div>
                            <div id="marker-overlay">
                                <div class="marker-item" draggable="true" data-type="start-zone">
                                    <img src="http://www.google.com/mapfiles/markerS.png" />
                                </div>
                                <div class="marker-item" draggable="true" data-type="end-zone">
                                    <img src="http://www.google.com/mapfiles/markerE.png" />
                                </div>
                            </div>
                        </div>
                    </div>
                    <div class="col-md-5" style="background-color: #f8f9fa; padding: 20px; overflow-y: auto;">
                        <div id="eventManagementFormContainer">
                            <!-- Form will be inserted here -->
                        </div>
                    </div>
                </div>
            </div>
        `;
    }

    var changeFields = function(toDisable) {
        var StartLatValue = $('#StartLat');
        var StartLonValue = $('#StartLon');
        var EndLatValue = $('#EndLat');
        var EndLonValue = $('#EndLon');
        var AdvisorySpeedValue = $('#AdvisorySpeed');

        if(toDisable == true) {
            StartLatValue.prop('disabled', true);
            StartLonValue.prop('disabled', true);
            EndLatValue.prop('disabled', true);
            EndLonValue.prop('disabled', true);
            AdvisorySpeedValue.prop('disabled', true);
        } else {
            StartLatValue.prop('disabled', false);
            StartLonValue.prop('disabled', false);
            EndLatValue.prop('disabled', false);
            EndLonValue.prop('disabled', false);
            AdvisorySpeedValue.prop('disabled', false);
        }
    }

    var changeBtn = function(btn, toStatus) {
        if(toStatus.toLowerCase().includes('start')) {
            $(btn).css('background-color', '#28a745');
            $(btn).text("Start Broadcast");
            $(btn).addClass('btn-success');
            $(btn).removeClass('btn-danger');
        } else {
            $(btn).css('background-color', '#dc3545');
            $(btn).text("Stop Broadcast");
            $(btn).removeClass('btn-success');
            $(btn).addClass('btn-danger');
        }
    }

    var deg2rad = function(deg) {
        return deg * Math.PI / 180;
    }

    var latLonToXY = function(lat, lon, originLat, originLon) {
        // Convert using flat earth approximation
        const R = 6371000; // Earth's radius in meters
        const x = R * deg2rad(lon - originLon) * Math.cos(deg2rad((lat + originLat) / 2));
        const y = R * deg2rad(lat - originLat);
        return [x, y];
    }

    var send_start_broadcasting_request = function (btn) {
        console.log("send start broadcasting request");

        // Validate ROS connection
        if (typeof ros === 'undefined' || !ros) {
            alert('ROS connection not available. Please ensure the system is connected.');
            return false;
        }

        var sendStartBCRequest = new ROSLIB.Service({
            ros: ros,
            name: '/start_broadcasting_traffic_event',
            serviceType: 'carma_msgs/srv/SetTrafficEvent.h'
        });

        // Validate form inputs
        var StartLatValue = $('#StartLat').val();
        var StartLonValue = $('#StartLon').val();
        var EndLatValue = $('#EndLat').val();
        var EndLonValue = $('#EndLon').val();
        var VehicleLatValue = $('#LatitudeSpan').text();
        var VehicleLonValue = $('#LongitudeSpan').text();
        var AdvisorySpeedValue = $('#AdvisorySpeed').val();

        // Input validation (using default ranges if CarmaJS.Config not available)
        var LatRange = { MIN: -90, MAX: 90 };
        var LonRange = { MIN: -180, MAX: 180 };
        var advisorySpeedRange = { MIN: 1, MAX: 80 };

        if (typeof CarmaJS !== 'undefined' && CarmaJS.Config) {
            try {
                LatRange = CarmaJS.Config.getLatitudeRange();
                LonRange = CarmaJS.Config.getLongitudeRange();
                advisorySpeedRange = CarmaJS.Config.getAdvisorySpeedRange();
            } catch (e) {
                console.warn('Could not get config ranges, using defaults');
            }
        }

        if(StartLatValue == "" || isNaN(StartLatValue) || StartLatValue > LatRange.MAX || StartLatValue < LatRange.MIN) {
            alert("Start latitude is required, should be between " + LatRange.MIN + " and " + LatRange.MAX);
            return false;
        }
        if(StartLonValue == "" || isNaN(StartLonValue) || StartLonValue > LonRange.MAX || StartLonValue < LonRange.MIN) {
            alert("Start longitude is required, should be between " + LonRange.MIN + " and " + LonRange.MAX);
            return false;
        }
        if(EndLatValue == "" || isNaN(EndLatValue) || EndLatValue > LatRange.MAX || EndLatValue < LatRange.MIN) {
            alert("End latitude is required, should be between " + LatRange.MIN + " and " + LatRange.MAX);
            return false;
        }
        if(EndLonValue == "" || isNaN(EndLonValue) || EndLonValue > LonRange.MAX || EndLonValue < LonRange.MIN) {
            alert("End longitude is required, should be between " + LonRange.MIN + " and " + LonRange.MAX);
            return false;
        }
        if(VehicleLatValue == "" || isNaN(VehicleLatValue) || VehicleLatValue > LatRange.MAX || VehicleLatValue < LatRange.MIN) {
            alert("Vehicle latitude is required, should be between " + LatRange.MIN + " and " + LatRange.MAX);
            return false;
        }
        if(VehicleLonValue == "" || isNaN(VehicleLonValue) || VehicleLonValue > LonRange.MAX || VehicleLonValue < LonRange.MIN) {
            alert("Vehicle longitude is required, should be between " + LonRange.MIN + " and " + LonRange.MAX);
            return false;
        }
        if(AdvisorySpeedValue == "" || isNaN(AdvisorySpeedValue) || AdvisorySpeedValue > advisorySpeedRange.MAX || AdvisorySpeedValue < advisorySpeedRange.MIN) {
            alert("Advisory Speed value is required, should be between " + advisorySpeedRange.MIN + " and " + advisorySpeedRange.MAX);
            return false;
        }
        // Project vehicle position onto the start and end line
        var startLat = parseFloat(StartLatValue);
        var startLon = parseFloat(StartLonValue);
        var endLat = parseFloat(EndLatValue);
        var endLon = parseFloat(EndLonValue);
        var vehicleLat = parseFloat(VehicleLatValue);
        var vehicleLon = parseFloat(VehicleLonValue);
        // Calculate the up track and down track based on the start and end coordinates
        const [endx, endy] = latLonToXY(endLat, endLon, startLat, startLon);
        const [vehx, vehy] = latLonToXY(vehicleLat, vehicleLon, startLat, startLon);
        console.log("End X: " + endx + ", End Y: " + endy);
        console.log("Vehicle X: " + vehx + ", Vehicle Y: " + vehy);

        const dx = endx;
        const dy = endy;
        const lengthSq = dx * dx + dy * dy;
        const segmentLength = Math.sqrt(lengthSq);

        // Project vehicle onto the segment
        const t = ((vehx * dx) + (vehy * dy)) / lengthSq;

        const projx = t * dx;
        const projy = t * dy;
        console.log("Projected X: " + projx + ", Projected Y: " + projy);

        const upTrack = t * segmentLength;

        // Downtrack: from projection to end, projected along segment
        const dx2 = projx - endx;
        const dy2 = projy - endy;
        const downTrack = -1 * (dx2 * dx + dy2 * dy) / segmentLength;

        // Compute minimum gap
        const minimumGap = 4.0;  // TODO: Get this using the closed lanes

        var request = new ROSLIB.ServiceRequest({
            up_track: parseFloat(upTrack),
            down_track: parseFloat(downTrack),
            minimum_gap: parseFloat(minimumGap),
            advisory_speed: parseFloat(AdvisorySpeedValue)
        });

        try {
            sendStartBCRequest.callService(request, function(result) {
                console.log('Broadcasting request result:', result);
            });
            subscribe_outgoing_mobility_operation();

        } catch(ex) {
            console.error("send start broadcasting request error: " + ex.message);
        }
    }

    // Subscribe to /gps_common_fix topic
    var subscribe_gps_common_fix = function() {
        console.log('subscribe_gps_common_fix called');

        if (typeof ros === 'undefined' || !ros) {
            console.warn('ROS not available for GPS subscription');
            return;
        }

        var listenerGPS = new ROSLIB.Topic({
            ros: ros,
            name: '/hardware_interface/gps_common_fix',
            messageType: 'gps_msgs/msg/GPSFix'
        });

        listenerGPS.subscribe(function (message) {
            console.log("GPS data received: " + JSON.stringify(message));
            if(message.latitude != null && message.latitude != 'undefined' && message.longitude != null && message.longitude != 'undefined') {
                $("#LatitudeSpan").text(message.latitude.toFixed(6));
                $("#LongitudeSpan").text(message.longitude.toFixed(6));
                $("#GPSStatusSpan").text("GOOD");
                $("#GPSStatusSpan").removeClass("text-danger").addClass("text-success");

                // Update host marker position on map if available
                if (typeof window.moveMarkerWithTimeout === 'function' && typeof window.hostmarker !== 'undefined' && window.hostmarker) {
                    window.moveMarkerWithTimeout(window.hostmarker, message.latitude, message.longitude, 0);
                }
            }
        });
    }

    // Subscribe to /outgoing_mobility_operation topic
    var subscribe_outgoing_mobility_operation = function() {
        if (typeof ros === 'undefined' || !ros) {
            console.warn('ROS not available for mobility operation subscription');
            return;
        }

        var listenerMobilityOperation = new ROSLIB.Topic({
            ros: ros,
            name: '/outgoing_mobility_operation',
            messageType: 'carma_v2x_msgs/msg/MobilityOperation'
        });

        listenerMobilityOperation.subscribe(function (message) {
            listenerMobilityOperation.unsubscribe();
            console.log("Mobility operation received: " + JSON.stringify(message.strategy_params));

            changeBtn($("#eventBroadcastBtn"), 'stop');
            changeFields(true);
            $('#BroadcastingFrequencySpan').text(BC_FQ_10HZ);
            $('#BroadcastingFrequencySpan').addClass('show');
            $('#BroadcastingFrequencyLabelTitle').addClass('show');
            $('#BroadcastingFrequencySpan').removeClass('hide');
            $('#BroadcastingFrequencyLabelTitle').removeClass('hide');

            broadcasting_btn_status = broadcasting_btn_status_t.STARTED;

            // Set session variables
            if (typeof EventSessionFormFields === 'undefined') {
                window.EventSessionFormFields = {};
            }
            EventSessionFormFields.BCStatus = 'true';

            if (message.strategy_params) {
                var params_arr = message.strategy_params.split(',');
                params_arr.forEach((item) => {
                    var keyValue = item.split(':');
                    if (keyValue.length >= 2) {
                        var key = keyValue[0].trim().toLowerCase();
                        var value = keyValue[1].trim();

                        switch(key) {
                            case 'startlat':
                                EventSessionFormFields.startLatitude = value;
                                break;
                            case 'startlon':
                                EventSessionFormFields.startLongitude = value;
                                break;
                            case 'endlat':
                                EventSessionFormFields.endLatitude = value;
                                break;
                            case 'endlon':
                                EventSessionFormFields.endLongitude = value;
                                break;
                            case 'advisory_speed':
                                EventSessionFormFields.advisorySpeed = value;
                                break;
                        }
                    }
                });
            }

            console.log('Updated EventSessionFormFields:', EventSessionFormFields);
        });
    }

    var send_stop_broadcasting_request = function (btn) {
        console.log("send stop broadcasting request");

        if (typeof ros === 'undefined' || !ros) {
            alert('ROS connection not available. Please ensure the system is connected.');
            return false;
        }

        var sendStopBCRequest = new ROSLIB.Service({
            ros: ros,
            name: '/stop_broadcasting_traffic_event',
            serviceType: 'std_srvs/srv/Trigger'
        });

        var request = new ROSLIB.ServiceRequest({});

        try {
            sendStopBCRequest.callService(request, function(result) {
                console.log("Stop broadcasting result: " + result.success);
                if(!result.success) {
                    console.log("Stop broadcasting failed");
                    alert('Failed to stop broadcasting: ' + (result.message || 'Unknown error'));
                } else {
                    console.log("Broadcasting stopped successfully");
                    changeBtn($("#eventBroadcastBtn"), 'start');
                    $('#BroadcastingFrequencySpan').addClass('hide');
                    $('#BroadcastingFrequencyLabelTitle').addClass('hide');
                    $('#BroadcastingFrequencySpan').removeClass('show');
                    $('#BroadcastingFrequencyLabelTitle').removeClass('show');
                    changeFields(false);

                    // Clear session variables
                    sessionStorage.removeItem('advisorySpeed');
                    sessionStorage.removeItem('startLatitude');
                    sessionStorage.removeItem('startLongitude');
                    sessionStorage.removeItem('endLatitude');
                    sessionStorage.removeItem('endLongitude');
                    sessionStorage.removeItem('BCStatus');

                    if (typeof EventSessionFormFields !== 'undefined') {
                        EventSessionFormFields.BCStatus = 'false';
                    }

                    // Clear geofence from map
                    if (typeof window.tcr_polygon !== 'undefined' && window.tcr_polygon) {
                        window.tcr_polygon.setMap(null);
                        window.tcr_polygon = null;
                    }
                }
            });
        } catch(ex) {
            console.error("send stop broadcasting request error: " + ex.message);
            alert('Error stopping broadcast: ' + ex.message);
        }
    }

    $.widget("CarmaJS.eventManagement", {
        _create: function () {
            console.log('Creating Event Management widget...');

            // Initialize the layout first
            initializeEventManagementLayout();

            // Get the form container
            var container = $('#eventManagementFormContainer');
            if (container.length === 0) {
                console.error('Form container not found');
                return;
            }

            // Create form elements
            this.createFormElements(container);

            // Check session variables after UI is created
            setTimeout(checkSessionVariables, 500);

            // Initialize map after a short delay to ensure container is ready
            setTimeout(function() {
                console.log('Triggering map initialization from widget...');
                ensureMapIsVisible();
            }, 1000);

            // Match form height to map height
            setTimeout(() => {
                const mapEl = document.getElementById('load-map');
                const formEl = document.getElementById('eventManagementFormContainer');

                if (mapEl && formEl) {
                    const formHeight = formEl.offsetHeight;
                    mapEl.style.height = `${formHeight}px`;

                    // Force map resize so it renders correctly
                    if (window.map) {
                        google.maps.event.trigger(window.map, 'resize');
                    }
                } else if (mapEl) {
                    console.warn('Could not match heights: form not found');
                } else if (formEl) {
                    console.warn('Could not match heights: map not found');
                } else {
                    console.warn('Could not match heights: map and form not found');
                }
            }, 5000);

            console.log("Event Management widget created");
        },

        createFormElements: function(container) {
            // Create the main title
            var titleElement = document.createElement('h2');
            titleElement.textContent = 'Event Management';
            titleElement.style.marginBottom = '20px';
            titleElement.style.color = '#333';
            container.append(titleElement);

            // Create form container
            var formContainer = document.createElement('div');
            formContainer.className = 'event-management-form';
            formContainer.style.backgroundColor = '#ffffff';
            formContainer.style.padding = '20px';
            formContainer.style.borderRadius = '8px';
            formContainer.style.boxShadow = '0 2px 4px rgba(0,0,0,0.1)';

            // Start zone (Up Track)
            var divStartZone = this.createFormRow(
                '<img src="http://www.google.com/mapfiles/markerS.png" style="height: 24px; vertical-align: middle; margin-right: 4px;" /> Start Zone Lat/Lon',
                ['StartLat', 'StartLon'],
                'number',
                '(degrees)',
                ['0', '0']
              );

            // End zone (Down Track)
            var divEndZone = this.createFormRow(
                '<img src="http://www.google.com/mapfiles/markerE.png" style="height: 24px; vertical-align: middle; margin-right: 4px;" /> End Zone Lat/Lon',
                ['EndLat', 'EndLon'],
                'number',
                '(degrees)',
                ['0', '0']
              );

            // Lanes blocked left
            var divLanesBlockedLeft = this.createFormRow('Lanes Blocked Left', ['LanesBlockedLeft'], 'number', '', ['0']);

            // Lanes blocked right
            var divLanesBlockedRight = this.createFormRow('Lanes Blocked Right', ['LanesBlockedRight'], 'number', '', ['0']);

            // Advisory Speed
            var divAdvisorySpeed = this.createFormRow('Advisory Speed:', ['AdvisorySpeed'], 'number', '(MPH)', ['15']);

            // Action Button
            var divActionBtn = this.createActionButton();

            // Current Position Section
            var divCurrentPosition = this.createCurrentPositionSection();

            // GPS Status
            var divGPSStatus = this.createGPSStatusSection();

            // Event Type
            var divEventType = this.createEventTypeSection();

            // Description
            var divDescription = this.createDescriptionSection();

            // Append all elements to form container
            formContainer.appendChild(divStartZone);
            formContainer.appendChild(divEndZone);
            formContainer.appendChild(divLanesBlockedLeft);
            formContainer.appendChild(divLanesBlockedRight);
            formContainer.appendChild(divAdvisorySpeed);
            formContainer.appendChild(divActionBtn);
            formContainer.appendChild(divCurrentPosition);
            formContainer.appendChild(divGPSStatus);
            formContainer.appendChild(divEventType);
            formContainer.appendChild(divDescription);

            container.append(formContainer);
        },

        createFormRow: function(labelText, fieldIds, inputType, unit, defaultValues) {
            var row = document.createElement('div');
            row.style.display = 'flex';
            row.style.alignItems = 'center';
            row.style.justifyContent = 'flex-start';
            row.style.width = '100%';
            row.style.maxWidth = '600px';
            row.style.marginBottom = '15px';
            row.style.fontSize = '14px';

            var label = document.createElement('label');
            label.innerHTML = labelText;
            label.style.width = '180px';
            label.style.marginRight = '10px';
            label.style.fontSize = '14px';
            label.style.flexShrink = '0';
            row.appendChild(label);

            var inputsContainer = document.createElement('div');
            inputsContainer.style.display = 'flex';
            inputsContainer.style.flexDirection = 'row';
            inputsContainer.style.justifyContent = 'center';
            inputsContainer.style.alignItems = 'center';
            inputsContainer.style.gap = '8px';
            inputsContainer.style.width = '300px';
            inputsContainer.style.gap = '8px';

            fieldIds.forEach((id, index) => {
                var input = document.createElement('input');
                input.type = inputType;
                input.id = id;
                input.value = defaultValues?.[index] || '';
                input.style.width = '120px';
                input.style.height = '30px';
                input.style.boxSizing = 'border-box';
                input.style.padding = '5px';
                input.style.fontSize = '14px';
                input.style.border = '1px solid #ccc';
                input.style.borderRadius = '4px';

                // Only add the negative number restriction for non-lat/lon fields
                if (!['StartLat', 'StartLon', 'EndLat', 'EndLon'].includes(id)) {
                    input.addEventListener('input', () => {
                        if (parseFloat(input.value) < 0) {
                            input.value = '0';
                        }
                    });
                }

                inputsContainer.appendChild(input);
            });

            row.appendChild(inputsContainer);

            var unitLabel = document.createElement('span');
            unitLabel.textContent = unit;
            unitLabel.style.color = '#666';
            unitLabel.style.fontSize = '12px';
            unitLabel.style.marginLeft = '10px';
            unitLabel.style.alignSelf = 'center';  // vertically center beside inputs
            row.appendChild(unitLabel);

            return row;
        },

        createActionButton: function() {
            var buttonContainer = document.createElement('div');
            buttonContainer.style.textAlign = 'center';
            buttonContainer.style.margin = '20px 0';

            var button = document.createElement('button');
            button.id = 'eventBroadcastBtn';
            button.type = 'button';
            button.textContent = 'Start Broadcast';
            button.style.backgroundColor = '#28a745';
            button.style.color = 'white';
            button.style.border = 'none';
            button.style.padding = '8px 16px';
            button.style.borderRadius = '4px';
            button.style.cursor = 'pointer';
            button.style.fontSize = '14px';

            button.addEventListener('click', function() {
                var buttonText = this.textContent.toLowerCase();
                if(buttonText.includes('start')) {
                    send_start_broadcasting_request(this);
                } else {
                    send_stop_broadcasting_request(this);
                }
            });

            buttonContainer.appendChild(button);
            return buttonContainer;
        },

        createCurrentPositionSection: function () {
            var section = document.createElement('div');
            section.style.marginTop = '20px';
            section.style.marginBottom = '15px';

            const createLine = (labelText, spanId, valueText) => {
                const line = document.createElement('div');
                line.style.fontSize = '14px';
                line.style.color = '#333';
                line.style.marginBottom = '8px';

                line.innerHTML = `${labelText} <span id="${spanId}" style="font-weight: bold; margin-left: 4px;">${valueText}</span>`;
                return line;
            };

            // TFHRC default coordinates
            var initialLat = 38.955882;
            var InitialLng = -77.147720;

            // Try to get coordinates from CarmaJS.Config
            if (typeof CarmaJS !== 'undefined' && CarmaJS.Config) {
                try {
                    initialLat = CarmaJS.Config.getInitialLatitude();
                    InitialLng = CarmaJS.Config.getInitialLongitude();
                } catch (e) {
                    console.warn('Could not get initial coordinates from config, using TFHRC defaults');
                }
            }

            const latLine = createLine('Current Latitude:', 'LatitudeSpan', initialLat.toString());
            const lonLine = createLine('Current Longitude:', 'LongitudeSpan', InitialLng.toString());

            section.appendChild(latLine);
            section.appendChild(lonLine);

            return section;
        },

        createGPSStatusSection: function() {
            var section = document.createElement('div');
            section.style.display = 'flex';
            section.style.alignItems = 'center';
            section.style.marginBottom = '15px';

            var icon = document.createElement('img');
            icon.src = 'http://www.google.com/mapfiles/ms/icons/blue-dot.png';
            icon.style.marginRight = '8px';
            icon.style.width = '20px';
            icon.style.height = '20px';
            icon.style.objectFit = 'contain';

            var label = document.createElement('span');
            label.textContent = 'GPS status:';
            label.style.marginRight = '10px';
            label.style.fontSize = '14px';

            var status = document.createElement('span');
            status.id = 'GPSStatusSpan';
            status.textContent = 'GOOD';
            status.style.color = '#28a745';
            status.style.fontWeight = 'bold';
            status.style.fontSize = '14px';

            section.appendChild(icon);
            section.appendChild(label);
            section.appendChild(status);

            return section;
        },

        createEventTypeSection: function() {
            var section = document.createElement('div');
            section.style.display = 'flex';
            section.style.alignItems = 'center';
            section.style.marginBottom = '15px';

            var label = document.createElement('span');
            label.textContent = 'Event Type:';
            label.style.marginRight = '10px';
            label.style.fontSize = '14px';

            var eventType = document.createElement('span');
            eventType.textContent = 'Move Over Law';
            eventType.style.fontSize = '14px';
            eventType.style.fontWeight = 'bold';

            section.appendChild(label);
            section.appendChild(eventType);

            return section;
        },

        createDescriptionSection: function() {
            var section = document.createElement('div');
            section.style.marginTop = '20px';
            section.style.padding = '15px';
            section.style.backgroundColor = '#f8f9fa';
            section.style.borderRadius = '4px';
            section.style.fontSize = '12px';
            section.style.lineHeight = '1.4';
            section.style.color = '#666';

            var description = document.createElement('div');
            description.innerHTML = `
                <b>This screen creates a geofence using the selected GPS locations to identify a closed segment on the road. Speed Advisory segments are automatically generated around the closed segment.</b><br><br>
                Start Zone: Distance between the selected GPS location and the beginning of the geofence<br><br>
                End Zone: Distance between the end of the geofence and the selected GPS location<br><br>
                Lanes Blocked: Number of lanes blocked to the left of the GPS location<br><br>
                Advisory Speed: Recommended speed within geo-fence, around the closed segment
            `;

            section.appendChild(description);

            return section;
        },

        subscribe_gps_common_fix: function() {
            subscribe_gps_common_fix();
        },

        _destroy: function () {
            this.element.empty();
            this._super();
            console.log("Event Management widget destroyed");
        }
    });

    var loadCustomWidget = function (container) {
        console.log('Loading Event Management widget...');
        try {
            // Create widget
            container.eventManagement();

            // Subscribe to GPS after a delay
            setTimeout(function() {
                container.eventManagement("subscribe_gps_common_fix");
            }, 2000);

        } catch (error) {
            console.error('Error loading Event Management widget:', error);
        }
    };

    //*** Public API  ***
    return {
        loadCustomWidget: loadCustomWidget,
        ensureMapIsVisible: ensureMapIsVisible
    };
})();
