/*
 * Copyright (C) 2018-2021 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/***
 This file shall contain Map related functions - Fixed version with Location Toggle and ROS Broadcasting
****/
// Load Material Icons font if not loaded already
if (!document.querySelector('link[href*="material-icons"]')) {
    const link = document.createElement('link');
    link.href = 'https://fonts.googleapis.com/icon?family=Material+Icons';
    link.rel = 'stylesheet';
    document.head.appendChild(link);
}

// Global variables
let map;
let markers = [];
let hostmarker;
let tcr_polygon;
let routePlanCoordinates;
let mapInitialized = false;
let placedMarkers = [];
let closedLabelOverlay = null;
let currentLocationButton = null;
let searchBox = null;
let searchInput = null;

// Location reference toggle variables
let useStartMarkerLocation = false;
let rosStartPublisher = null;
let rosEndPublisher = null;
let publishInterval = null;

// Polygon types
const g_polygon_type = {
    TCR: 'TCR'
};

$(document).ready(() => {
    // Don't initialize map immediately, wait for container to be ready
    console.log('Document ready, waiting for map container...');
    initializeROS();
});

// Initialize ROS connection for GPS broadcasting
function initializeROS() {
    try {
        // Initialize ROS connection (assumes rosbridge_websocket is available)
        const ros = new ROSLIB.Ros({
            url: 'ws://localhost:9090' // Adjust URL as needed
        });

        ros.on('connection', function() {
            console.log('Connected to ROS websocket server.');
        });

        ros.on('error', function(error) {
            console.log('Error connecting to ROS websocket server: ', error);
        });

        ros.on('close', function() {
            console.log('Connection to ROS websocket server closed.');
        });

        // Create GPS Fix publishers for both start and end locations
        rosStartPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/gps_fix_start_zone',
            messageType: 'sensor_msgs/NavSatFix'
        });

        rosEndPublisher = new ROSLIB.Topic({
            ros: ros,
            name: '/gps_fix_end_zone',
            messageType: 'sensor_msgs/NavSatFix'
        });

        console.log('ROS GPS publishers initialized for start and end zones');
    } catch (error) {
        console.error('Failed to initialize ROS connection:', error);
        console.log('ROS functionality will be disabled');
    }
}

// Function to create current location button
function createCurrentLocationButton() {
    const locationButton = document.createElement('button');
    locationButton.id = 'current-location-btn';
    locationButton.innerHTML = '<span class="material-icons" style="font-size: 18px; color: #333;">my_location</span>';
    locationButton.title = 'Go to my current location';
    locationButton.style.cssText = `
        background: white;
        border: 2px solid #dadce0;
        border-radius: 3px;
        cursor: pointer;
        font-size: 16px;
        font-weight: bold;
        font-family: Arial, sans-serif;
        height: 40px;
        width: 40px;
        margin: 10px;
        padding: 0;
        text-align: center;
        box-shadow: 0 2px 6px rgba(0,0,0,0.3);
        transition: all 0.2s ease;
    `;

    // Add hover effects
    locationButton.addEventListener('mouseenter', function() {
        this.style.backgroundColor = '#f1f3f4';
        this.style.borderColor = '#c1c7cd';
    });

    locationButton.addEventListener('mouseleave', function() {
        this.style.backgroundColor = 'white';
        this.style.borderColor = '#dadce0';
    });

    locationButton.addEventListener('click', function() {
        getCurrentLocationAndUpdateHost();
    });

    return locationButton;
}

// Function to create location search bar
function createLocationSearchBar() {
    const searchContainer = document.createElement('div');
    searchContainer.id = 'search-container';
    searchContainer.style.cssText = `
        background: white;
        border: 2px solid #dadce0;
        border-radius: 3px;
        margin: 10px;
        padding: 0;
        box-shadow: 0 2px 6px rgba(0,0,0,0.3);
        min-width: 250px;
    `;

    searchInput = document.createElement('input');
    searchInput.id = 'location-search-input';
    searchInput.type = 'text';
    searchInput.placeholder = 'Search for a location...';
    searchInput.style.cssText = `
        border: none;
        outline: none;
        padding: 10px 15px;
        font-size: 14px;
        font-family: Arial, sans-serif;
        width: 100%;
        box-sizing: border-box;
    `;

    searchContainer.appendChild(searchInput);
    return searchContainer;
}

// Function to initialize search functionality
function initializeLocationSearch() {
    if (!searchInput || !map) return;

    searchBox = new google.maps.places.SearchBox(searchInput);

    // Bias the SearchBox results towards current map's viewport
    map.addListener('bounds_changed', () => {
        searchBox.setBounds(map.getBounds());
    });

    searchBox.addListener('places_changed', () => {
        const places = searchBox.getPlaces();

        if (places.length === 0) {
            return;
        }

        const place = places[0];

        if (!place.geometry || !place.geometry.location) {
            console.log("No location data for: " + place.name);
            return;
        }

        // Update host marker to searched location
        const lat = place.geometry.location.lat();
        const lng = place.geometry.location.lng();

        // Update the latitude and longitude span elements
        $('#LatitudeSpan').text(lat.toFixed(6));
        $('#LongitudeSpan').text(lng.toFixed(6));

        // Update host marker
        updateHostMarkerToCurrentLocation(lat, lng);

        // Center map on searched location
        map.setCenter(place.geometry.location);
        map.setZoom(17);

        // Show success message
        showLocationMessage(`Moved to: ${place.name}`, 'success');

        // Clear search input
        searchInput.value = '';
    });
}

// Function to get current location and update host marker
function getCurrentLocationAndUpdateHost() {
    if (!navigator.geolocation) {
        alert('Geolocation is not supported by this browser.');
        return;
    }

    // Change button appearance while loading
    const button = document.getElementById('current-location-btn');
    if (button) {
        button.innerHTML = '<span class="material-icons" style="font-size: 18px; color: #333;">my_location</span>';
        button.disabled = true;
        button.style.cursor = 'not-allowed';
    }

    const options = {
        enableHighAccuracy: true,
        timeout: 10000,
        maximumAge: 60000 // Cache location for 1 minute
    };

    navigator.geolocation.getCurrentPosition(
        function(position) {
            const currentLat = position.coords.latitude;
            const currentLng = position.coords.longitude;

            console.log('Current location obtained:', currentLat, currentLng);

            // Update the latitude and longitude span elements
            $('#LatitudeSpan').text(currentLat.toFixed(6));
            $('#LongitudeSpan').text(currentLng.toFixed(6));

            // Update or create host marker
            updateHostMarkerToCurrentLocation(currentLat, currentLng);

            // Center map on current location
            if (map) {
                map.setCenter({ lat: currentLat, lng: currentLng });
                map.setZoom(17); // Set appropriate zoom level
            }

            // Show success message
            showLocationMessage('Location updated successfully!', 'success');

            // Reset button appearance
            if (button) {
                button.innerHTML = '<span class="material-icons" style="font-size: 18px; color: #333;">my_location</span>';
                button.disabled = false;
                button.style.cursor = 'pointer';
            }
        },
        function(error) {
            let errorMessage = 'Unable to get your location. ';

            switch(error.code) {
                case error.PERMISSION_DENIED:
                    errorMessage += 'Location access denied by user.';
                    break;
                case error.POSITION_UNAVAILABLE:
                    errorMessage += 'Location information unavailable.';
                    break;
                case error.TIMEOUT:
                    errorMessage += 'Location request timed out.';
                    break;
                default:
                    errorMessage += 'An unknown error occurred.';
                    break;
            }

            console.error('Geolocation error:', error);
            showLocationMessage(errorMessage, 'error');

            // Reset button appearance
            if (button) {
                button.innerHTML = '<span class="material-icons" style="font-size: 18px; color: #666; opacity: 0.5;">gps_not_fixed</span>';
                button.disabled = false;
                button.style.cursor = 'pointer';
            }
        },
        options
    );
}

// Function to update host marker to current location
function updateHostMarkerToCurrentLocation(lat, lng) {
    if (!map) {
        console.error('Map not initialized when trying to update host marker');
        return;
    }

    // Remove existing host marker if it exists
    if (hostmarker) {
        hostmarker.setMap(null);
    }

    // Create new host marker at current location
    hostmarker = new google.maps.Marker({
        id: 'mHostVehicle',
        position: { lat: lat, lng: lng },
        map: map,
        icon: 'http://www.google.com/mapfiles/ms/icons/blue-dot.png',
        title: 'Host Vehicle (Current Location)',
        zIndex: 1
    });

    window.hostmarker = hostmarker;

    // Add a subtle animation to the marker
    hostmarker.setAnimation(google.maps.Animation.DROP);

    // Stop animation after a brief moment
    setTimeout(() => {
        if (hostmarker) {
            hostmarker.setAnimation(null);
        }
    }, 1000);
}

// Function to show location status messages
function showLocationMessage(message, type) {
    // Remove existing message if any
    const existingMessage = document.getElementById('location-message');
    if (existingMessage) {
        existingMessage.remove();
    }

    const messageDiv = document.createElement('div');
    messageDiv.id = 'location-message';
    messageDiv.textContent = message;

    const isError = type === 'error';
    messageDiv.style.cssText = `
        position: fixed;
        top: 20px;
        left: 50%;
        transform: translateX(-50%);
        background: ${isError ? '#ff4444' : '#4CAF50'};
        color: white;
        padding: 12px 24px;
        border-radius: 6px;
        font-size: 14px;
        font-weight: 500;
        box-shadow: 0 4px 12px rgba(0,0,0,0.3);
        z-index: 10000;
        animation: slideInFromTop 0.3s ease;
    `;

    // Add CSS animation keyframes
    if (!document.getElementById('location-message-styles')) {
        const style = document.createElement('style');
        style.id = 'location-message-styles';
        style.textContent = `
            @keyframes slideInFromTop {
                0% {
                    transform: translateX(-50%) translateY(-100%);
                    opacity: 0;
                }
                100% {
                    transform: translateX(-50%) translateY(0);
                    opacity: 1;
                }
            }
        `;
        document.head.appendChild(style);
    }

    document.body.appendChild(messageDiv);

    // Auto-remove message after 3 seconds
    setTimeout(() => {
        if (messageDiv && messageDiv.parentNode) {
            messageDiv.style.animation = 'slideInFromTop 0.3s ease reverse';
            setTimeout(() => {
                messageDiv.remove();
            }, 300);
        }
    }, 3000);
}

// Function to create location reference toggle slider
function createLocationToggleSlider() {
    const toggleContainer = document.createElement('div');
    toggleContainer.id = 'location-toggle-container';
    toggleContainer.style.cssText = `
        background: white;
        padding: 15px;
        border-radius: 5px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.3);
        font-family: Arial, sans-serif;
        min-width: 280px;
        margin: 10px;
    `;

    const toggleLabel = document.createElement('label');
    toggleLabel.style.cssText = `
        display: block;
        margin-bottom: 10px;
        font-weight: bold;
        font-size: 14px;
        color: #333;
    `;
    toggleLabel.textContent = 'Reference Location Source:';

    const sliderContainer = document.createElement('div');
    sliderContainer.style.cssText = `
        display: flex;
        align-items: center;
        gap: 10px;
        margin-bottom: 10px;
    `;

    const torcLabel = document.createElement('span');
    torcLabel.textContent = 'TORC Pinpoint';
    torcLabel.style.cssText = `
        font-size: 12px;
        color: #666;
        min-width: 80px;
    `;

    const slider = document.createElement('input');
    slider.type = 'range';
    slider.id = 'location-toggle-slider';
    slider.min = '0';
    slider.max = '1';
    slider.value = '0';
    slider.step = '1';
    slider.style.cssText = `
        flex: 1;
        margin: 0 10px;
    `;

    const startLabel = document.createElement('span');
    startLabel.textContent = 'Start/End Markers';
    startLabel.style.cssText = `
        font-size: 12px;
        color: #666;
        min-width: 100px;
    `;

    const statusDiv = document.createElement('div');
    statusDiv.id = 'location-status';
    statusDiv.style.cssText = `
        font-size: 12px;
        color: #007bff;
        margin-top: 5px;
        font-weight: bold;
    `;
    statusDiv.textContent = 'Using: TORC Pinpoint Driver';

    const rosStatusDiv = document.createElement('div');
    rosStatusDiv.id = 'ros-status';
    rosStatusDiv.style.cssText = `
        font-size: 11px;
        color: #28a745;
        margin-top: 5px;
    `;
    rosStatusDiv.textContent = 'ROS: Ready';

    const detailsDiv = document.createElement('div');
    detailsDiv.id = 'broadcast-details';
    detailsDiv.style.cssText = `
        font-size: 10px;
        color: #6c757d;
        margin-top: 5px;
        font-style: italic;
    `;
    detailsDiv.textContent = '';

    sliderContainer.appendChild(torcLabel);
    sliderContainer.appendChild(slider);
    sliderContainer.appendChild(startLabel);

    toggleContainer.appendChild(toggleLabel);
    toggleContainer.appendChild(sliderContainer);
    toggleContainer.appendChild(statusDiv);
    toggleContainer.appendChild(rosStatusDiv);
    toggleContainer.appendChild(detailsDiv);

    // Add event listener for slider
    slider.addEventListener('input', function() {
        const value = parseInt(this.value);
        useStartMarkerLocation = value === 1;
        updateLocationStatus();
        handleLocationToggle();
    });

    return toggleContainer;
}

// Update location status display
function updateLocationStatus() {
    const statusDiv = document.getElementById('location-status');
    const rosStatusDiv = document.getElementById('ros-status');
    const detailsDiv = document.getElementById('broadcast-details');

    if (statusDiv) {
        if (useStartMarkerLocation) {
            statusDiv.textContent = 'Using: Start/End Marker Locations';
            statusDiv.style.color = '#ff6b35';
        } else {
            statusDiv.textContent = 'Using: TORC Pinpoint Driver';
            statusDiv.style.color = '#007bff';
        }
    }

    if (rosStatusDiv) {
        if (useStartMarkerLocation && rosStartPublisher && rosEndPublisher) {
            rosStatusDiv.textContent = 'ROS: Broadcasting GPS Fix (Start & End)';
            rosStatusDiv.style.color = '#28a745';
        } else if (useStartMarkerLocation && (!rosStartPublisher || !rosEndPublisher)) {
            rosStatusDiv.textContent = 'ROS: Connection Error';
            rosStatusDiv.style.color = '#dc3545';
        } else {
            rosStatusDiv.textContent = 'ROS: Standby';
            rosStatusDiv.style.color = '#6c757d';
        }
    }

    if (detailsDiv) {
        if (useStartMarkerLocation) {
            detailsDiv.textContent = 'Topics: /gps_fix_start_zone & /gps_fix_end_zone @ 10Hz';
        } else {
            detailsDiv.textContent = '';
        }
    }
}

// Handle location toggle change
function handleLocationToggle() {
    if (useStartMarkerLocation) {
        startGPSBroadcasting();
    } else {
        stopGPSBroadcasting();
    }
}

// Start broadcasting GPS Fix messages
function startGPSBroadcasting() {
    if (!rosStartPublisher || !rosEndPublisher) {
        console.error('ROS publishers not available');
        return;
    }

    // Clear any existing interval
    if (publishInterval) {
        clearInterval(publishInterval);
    }

    // Start publishing GPS Fix messages at 10Hz
    publishInterval = setInterval(() => {
        const startLat = parseFloat(document.getElementById('StartLat')?.value || '0');
        const startLon = parseFloat(document.getElementById('StartLon')?.value || '0');
        const endLat = parseFloat(document.getElementById('EndLat')?.value || '0');
        const endLon = parseFloat(document.getElementById('EndLon')?.value || '0');

        const timestamp = {
            sec: Math.floor(Date.now() / 1000),
            nsec: (Date.now() % 1000) * 1000000
        };

        // Publish start zone GPS fix
        if (startLat !== 0 && startLon !== 0) {
            const startGpsFixMessage = new ROSLIB.Message({
                header: {
                    stamp: timestamp,
                    frame_id: 'start_zone_gps'
                },
                status: {
                    status: 0, // STATUS_FIX
                },
                latitude: startLat,
                longitude: startLon,
                altitude: 0.0,
                position_covariance: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
                position_covariance_type: 1 // COVARIANCE_TYPE_KNOWN
            });

            rosStartPublisher.publish(startGpsFixMessage);
            console.log('Published Start GPS Fix:', startLat, startLon);
        }

        // Publish end zone GPS fix
        if (endLat !== 0 && endLon !== 0) {
            const endGpsFixMessage = new ROSLIB.Message({
                header: {
                    stamp: timestamp,
                    frame_id: 'end_zone_gps'
                },
                status: {
                    status: 0, // STATUS_FIX
                },
                latitude: endLat,
                longitude: endLon,
                altitude: 0.0,
                position_covariance: [
                    1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0
                ],
                position_covariance_type: 1 // COVARIANCE_TYPE_KNOWN
            });

            rosEndPublisher.publish(endGpsFixMessage);
            console.log('Published End GPS Fix:', endLat, endLon);
        }
    }, 100); // 10Hz (100ms interval)

    console.log('Started GPS broadcasting from Start and End marker locations');
}

// Stop broadcasting GPS Fix messages
function stopGPSBroadcasting() {
    if (publishInterval) {
        clearInterval(publishInterval);
        publishInterval = null;
    }
    console.log('Stopped GPS broadcasting for both start and end zones');
}

// Function to initialize map when container is ready and visible
function initializeMapWhenReady() {
    const mapContainer = document.getElementById('load-map');

    if (!mapContainer) {
        console.log('Map container not found, retrying in 1 second...');
        setTimeout(initializeMapWhenReady, 1000);
        return;
    }

    // Check if container is visible and has dimensions
    const containerRect = mapContainer.getBoundingClientRect();
    if (containerRect.width === 0 || containerRect.height === 0) {
        console.log('Map container has no dimensions, retrying in 1 second...');
        setTimeout(initializeMapWhenReady, 1000);
        return;
    }

    if (mapInitialized) {
        console.log('Map already initialized');
        return;
    }

    console.log('Map container ready, initializing map...');
    initMap();
}

// Simplified initMap function without iframe
async function initMap() {
    if (mapInitialized) {
        return;
    }

    try {
        // Load API key first
        const apiKey = await loadApiKey();

        // Create script tag to load Google Maps API
        const script = document.createElement('script');
        script.src = `https://maps.googleapis.com/maps/api/js?key=${apiKey}&libraries=places&loading=async&callback=showNewMap`;;
        script.async = true;
        script.defer = true;
        document.head.appendChild(script);

        // Make showNewMap globally available
        window.showNewMap = showNewMap;

    } catch (error) {
        console.error('Failed to initialize map:', error);
        // Show error message in map container
        const mapContainer = document.getElementById('load-map');
        if (mapContainer) {
            mapContainer.innerHTML = '<div style="padding: 20px; text-align: center; color: red; background: #f8f9fa; border: 1px solid #ddd; border-radius: 5px;">Failed to load Google Maps. Please check your API key.</div>';
        }
    }
}

// Callback function for Google Maps API
function showNewMap() {
    const mapContainer = document.getElementById('load-map');
    if (!mapContainer) {
        console.error('Map container not found during callback');
        return;
    }

    // Ensure map container is properly styled for flex layout
    const mapWrapper = document.getElementById('map-wrapper');
    if (!mapWrapper) {
        console.warn('Map wrapper not found. Make sure HTML uses a wrapper with #map-wrapper ID.');
    }

    if (mapContainer.offsetWidth === 0 || mapContainer.offsetHeight === 0) {
        console.warn('Map container has no dimensions, setting default size');
        mapContainer.style.width = '100%';
        mapContainer.style.height = '400px';
    }

    try {
        map = new google.maps.Map(mapContainer, {
            zoom: 17,
            center: { lat: 38.955097, lng: -77.147190 },
            mapTypeId: 'hybrid',
            disableDefaultUI: true,
            zoomControl: true,
            zoomControlOptions: {
                position: google.maps.ControlPosition.LEFT_CENTER
            },
            scaleControl: true,
            mapTypeControl: true,
            fullscreenControl: true
        });

        // Add location toggle slider to map
        const toggleSlider = createLocationToggleSlider();
        map.controls[google.maps.ControlPosition.BOTTOM_RIGHT].push(toggleSlider);

        // Add current location button to map
        currentLocationButton = createCurrentLocationButton();
        map.controls[google.maps.ControlPosition.TOP_LEFT].push(currentLocationButton);

        // Add location search bar to map
        const searchBar = createLocationSearchBar();
        map.controls[google.maps.ControlPosition.TOP_CENTER].push(searchBar);

        google.maps.event.addListenerOnce(map, 'idle', function () {
            console.log('Google Maps fully loaded and ready');
            mapInitialized = true;

            const savedMarkers = sessionStorage.getItem('mapMarkers');
            if (savedMarkers) {
                try {
                    markers = JSON.parse(savedMarkers);
                } catch (e) {
                    console.warn('Failed to parse saved markers:', e);
                    markers = [];
                }
            }

            setRouteMap(map);
            setHostMarker();
            initializeLocationSearch();

            window.map = map;
            window.markers = markers;
            window.hostmarker = hostmarker;
            window.tcr_polygon = tcr_polygon;
            window.g_polygon_type = g_polygon_type;

            setTimeout(() => {
                google.maps.event.trigger(map, 'resize');
            }, 100);
        });

        setupDragAndDrop(map);

        console.log('Google Maps initialized successfully');

    } catch (error) {
        console.error('Error creating Google Map:', error);
        mapContainer.innerHTML = '<div style="padding: 20px; text-align: center; color: red;">Error creating map: ' + error.message + '</div>';
    }
}

function setupDragAndDrop(map) {
    const mapDiv = document.getElementById('load-map');
    // const placedMarkers = [];
    const markerTypesPlaced = {
        'start-zone': null,
        'end-zone': null
    };

    let rectangleOverlay = null; // store the rectangle
    let currentlyHighlighted = null;

    mapDiv.addEventListener('dragover', (e) => {
        e.preventDefault();
    });

    document.getElementById('StartLat').addEventListener('input', () => {
        updateStartZoneMarkerFromForm();
        // If using start marker location, update GPS broadcasting
        if (useStartMarkerLocation) {
            // GPS broadcasting will automatically use the updated coordinates
        }
    });

    document.getElementById('StartLon').addEventListener('input', () => {
        updateStartZoneMarkerFromForm();
        // If using start marker location, update GPS broadcasting
        if (useStartMarkerLocation) {
            // GPS broadcasting will automatically use the updated coordinates
        }
    });

    document.getElementById('EndLat').addEventListener('input', () => {
        updateEndZoneMarkerFromForm();
        // If using start/end marker locations, the GPS broadcasting will use updated coordinates
    });

    document.getElementById('EndLon').addEventListener('input', () => {
        updateEndZoneMarkerFromForm();
        // If using start/end marker locations, the GPS broadcasting will use updated coordinates
    });
    document.getElementById('LanesBlockedLeft').addEventListener('input', updateEndZoneMarkerFromForm);
    document.getElementById('LanesBlockedRight').addEventListener('input', updateEndZoneMarkerFromForm);
    document.getElementById('AdvisorySpeed').addEventListener('input', updateEndZoneMarkerFromForm);
    document.getElementById('StartLat').addEventListener('blur', updateMarkersFromInputs);
    document.getElementById('StartLon').addEventListener('blur', updateMarkersFromInputs);
    document.getElementById('EndLat').addEventListener('blur', updateMarkersFromInputs);
    document.getElementById('EndLon').addEventListener('blur', updateMarkersFromInputs);

    function updateMarkersFromInputs() {
        const startLat = parseFloat(document.getElementById('StartLat').value);
        const startLon = parseFloat(document.getElementById('StartLon').value);
        const endLat = parseFloat(document.getElementById('EndLat').value);
        const endLon = parseFloat(document.getElementById('EndLon').value);

        // Create or update start marker
        if (!isNaN(startLat) && !isNaN(startLon)) {
            if (markerTypesPlaced['start-zone']) {
                markerTypesPlaced['start-zone'].setPosition({lat: startLat, lng: startLon});
            } else {
                const startMarker = new google.maps.Marker({
                    position: {lat: startLat, lng: startLon},
                    map: map,
                    draggable: true,
                    title: 'start-zone',
                    icon: getMarkerIcon('start-zone')
                });
                startMarker.markerType = 'start-zone';
                markerTypesPlaced['start-zone'] = startMarker;
                placedMarkers.push(startMarker);

                startMarker.addListener('dragend', () => {
                    document.getElementById('StartLat').value = startMarker.getPosition().lat().toFixed(6);
                    document.getElementById('StartLon').value = startMarker.getPosition().lng().toFixed(6);
                    if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
                        drawRectangleFromStartToEnd(markerTypesPlaced['start-zone'], markerTypesPlaced['end-zone']);
                    }
                });
            }
        }

        // Create or update end marker
        if (!isNaN(endLat) && !isNaN(endLon)) {
            if (markerTypesPlaced['end-zone']) {
                markerTypesPlaced['end-zone'].setPosition({lat: endLat, lng: endLon});
            } else {
                const endMarker = new google.maps.Marker({
                    position: {lat: endLat, lng: endLon},
                    map: map,
                    draggable: true,
                    title: 'end-zone',
                    icon: getMarkerIcon('end-zone')
                });
                endMarker.markerType = 'end-zone';
                markerTypesPlaced['end-zone'] = endMarker;
                placedMarkers.push(endMarker);

                endMarker.addListener('dragend', () => {
                    document.getElementById('EndLat').value = endMarker.getPosition().lat().toFixed(6);
                    document.getElementById('EndLon').value = endMarker.getPosition().lng().toFixed(6);
                    if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
                        drawRectangleFromStartToEnd(markerTypesPlaced['start-zone'], markerTypesPlaced['end-zone']);
                    }
                });
            }
        }

        // Draw rectangles if both markers exist
        if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
            drawRectangleFromStartToEnd(markerTypesPlaced['start-zone'], markerTypesPlaced['end-zone']);
        }
    }

    mapDiv.addEventListener('drop', (e) => {
        e.preventDefault();

        const markerType = e.dataTransfer.getData("text/plain");
        const mapBounds = map.getDiv().getBoundingClientRect();

        if (markerType in markerTypesPlaced === false) {
            console.warn("Unknown marker type: ${markerType}");
            return;
        } else if (markerTypesPlaced[markerType]) {
            console.warn("Marker of type ${markerType} already placed");
            return;
        }

        const point = {
            x: e.clientX - mapBounds.left,
            y: e.clientY - mapBounds.top
        };

        const latLng = getLatLngFromPoint(point, map);
        if (!latLng) return;

        const marker = new google.maps.Marker({
            position: latLng,
            map: map,
            draggable: true,
            title: markerType,
            icon: getMarkerIcon(markerType)
        });

        marker.markerType = markerType;

        marker.markerType = markerType;

        // Store marker by type
        if (markerType in markerTypesPlaced) {
            markerTypesPlaced[markerType] = marker;
        }

        // Update the appropriate lat/lon fields
        if (markerType === 'start-zone') {
            document.getElementById('StartLat').value = latLng.lat().toFixed(6);
            document.getElementById('StartLon').value = latLng.lng().toFixed(6);
        }
        else if (markerType === 'end-zone') {
            document.getElementById('EndLat').value = latLng.lat().toFixed(6);
            document.getElementById('EndLon').value = latLng.lng().toFixed(6);
        }

        // Check if all three types are placed
        if (Object.values(markerTypesPlaced).every(m => m !== null)) {
            drawRectangleFromStartToEnd(markerTypesPlaced['start-zone'], markerTypesPlaced['end-zone']);
        }

        marker.addListener('dragend', () => {
            if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
                drawRectangleFromStartToEnd(
                    markerTypesPlaced['start-zone'],
                    markerTypesPlaced['end-zone']
                );
            }
            // Update the appropriate lat/lon fields
            if (marker.markerType === 'start-zone') {
                document.getElementById('StartLat').value = marker.getPosition().lat().toFixed(6);
                document.getElementById('StartLon').value = marker.getPosition().lng().toFixed(6);
            } else if (marker.markerType === 'end-zone') {
                document.getElementById('EndLat').value = marker.getPosition().lat().toFixed(6);
                document.getElementById('EndLon').value = marker.getPosition().lng().toFixed(6);
            }
        });

        marker.addListener('click', (e) => {
            e.domEvent?.stopPropagation?.();
            highlightMarker(marker);
        });

        placedMarkers.push(marker);
    });

    // Listen for clicks on the map itself to clear highlight
    map.addListener('click', () => {
        if (currentlyHighlighted) {
            currentlyHighlighted.setIcon(getMarkerIcon(currentlyHighlighted.markerType));
            currentlyHighlighted = null;
        }
    });

    document.querySelectorAll('.marker-item').forEach((el) => {
        el.addEventListener('dragstart', (e) => {
            e.dataTransfer.setData("text/plain", el.dataset.type);
        });
    });

    function highlightMarker(marker) {
        if (currentlyHighlighted === marker) return; // already highlighted

        if (currentlyHighlighted) {
            currentlyHighlighted.setIcon(getMarkerIcon(currentlyHighlighted.markerType));
        }

        marker.setIcon(getHighlightedIcon(marker.markerType));
        currentlyHighlighted = marker;
    }

    function updateStartZoneMarkerFromForm() {
        const lat = parseFloat(document.getElementById('StartLat').value);
        const lng = parseFloat(document.getElementById('StartLon').value);
        if (!isNaN(lat) && !isNaN(lng) && markerTypesPlaced['start-zone']) {
            const newPos = new google.maps.LatLng(lat, lng);
            markerTypesPlaced['start-zone'].setPosition(newPos);
            if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
                drawRectangleFromStartToEnd(
                    markerTypesPlaced['start-zone'],
                    markerTypesPlaced['end-zone']
                );
            }
        }
    }

    function updateEndZoneMarkerFromForm() {
        const lat = parseFloat(document.getElementById('EndLat').value);
        const lng = parseFloat(document.getElementById('EndLon').value);
        if (!isNaN(lat) && !isNaN(lng) && markerTypesPlaced['end-zone']) {
            const newPos = new google.maps.LatLng(lat, lng);
            markerTypesPlaced['end-zone'].setPosition(newPos);
            if (markerTypesPlaced['start-zone'] && markerTypesPlaced['end-zone']) {
                drawRectangleFromStartToEnd(
                    markerTypesPlaced['start-zone'],
                    markerTypesPlaced['end-zone']
                );
            }
        }
    }

    let closedLabelOverlay = null;

    class ClosedTextOverlay extends google.maps.OverlayView {
        constructor(position, angle) {
            super();
            this.position = position;
            this.angle = angle;
            this.div = null;
        }

        onAdd() {
            this.div = document.createElement('div');
            this.div.className = 'closed-label';
            this.div.innerText = 'CLOSED';

            const panes = this.getPanes();
            panes.overlayLayer.appendChild(this.div);
        }

        draw() {
            const projection = this.getProjection();
            const point = projection.fromLatLngToDivPixel(this.position);

            if (point && this.div) {
                this.div.style.left = point.x + 'px';
                this.div.style.top = point.y + 'px';
                this.div.style.position = 'absolute';
                this.div.style.transform = `translate(-50%, -50%) rotate(${this.angle}deg)`;
            }
        }

        onRemove() {
            if (this.div) {
                this.div.parentNode.removeChild(this.div);
                this.div = null;
            }
        }
    }

    class SpeedAdvisoryTextOverlay extends google.maps.OverlayView {
        constructor(position, angle) {
            super();
            this.position = position;
            this.angle = angle;
            this.div = null;
        }

        onAdd() {
            this.div = document.createElement('div');
            this.div.className = 'speed-advisory-label';
            var advisorySpeed = document.getElementById('AdvisorySpeed').value;
            this.div.innerText = advisorySpeed ? `${advisorySpeed} MPH` : 'N/A MPH';

            const panes = this.getPanes();
            panes.overlayLayer.appendChild(this.div);
        }

        draw() {
            const projection = this.getProjection();
            const point = projection.fromLatLngToDivPixel(this.position);

            if (point && this.div) {
                this.div.style.left = point.x + 'px';
                this.div.style.top = point.y + 'px';
                this.div.style.position = 'absolute';
                this.div.style.transform = `translate(-50%, -50%) rotate(${this.angle}deg)`;
            }
        }

        onRemove() {
            if (this.div) {
                this.div.parentNode.removeChild(this.div);
                this.div = null;
            }
        }
    }

    function drawRectanglesFromMarkers() {
        if (window.startMarker && window.endMarker) {
            drawRectangleFromStartToEnd(window.startMarker, window.endMarker);
        }
    }

    function drawRectangleFromStartToEnd(startMarker, endMarker) {
        const R = 6378137; // Earth's radius in meters

        const toRadians = deg => deg * Math.PI / 180;
        const toDegrees = rad => rad * 180 / Math.PI;

        const start = startMarker.getPosition();
        const end = endMarker.getPosition();

        const lat1 = toRadians(start.lat());
        const lng1 = toRadians(start.lng());
        const lat2 = toRadians(end.lat());
        const lng2 = toRadians(end.lng());

        // Cartesian coordinates
        const x1 = R * lng1 * Math.cos((lat1 + lat2) / 2);
        const y1 = R * lat1;
        const x2 = R * lng2 * Math.cos((lat1 + lat2) / 2);
        const y2 = R * lat2;

        // Vector and unit perpendicular vector
        const dx = x2 - x1;
        const dy = y2 - y1;
        const length = Math.sqrt(dx * dx + dy * dy);
        const px = -dy / length;
        const py = dx / length;

        const laneWidth = 4; // meters
        const lanesLeft = parseInt(document.getElementById('LanesBlockedLeft')?.value || '0', 10);
        const lanesRight = parseInt(document.getElementById('LanesBlockedRight')?.value || '0', 10);

        // Clear previous overlays
        if (window.rectangleOverlays) {
            window.rectangleOverlays.forEach(p => p.setMap(null));
        }
        window.rectangleOverlays = [];

        // Build list of lane indices: [-2, -1, 0, 1, 2] for example
        const laneIndices = [];

        for (let i = -lanesLeft - 1; i < 0; i++) laneIndices.push(i);  // left lanes
        laneIndices.push(0);                                       // center closed lane
        for (let i = 1; i <= lanesRight + 1; i++) laneIndices.push(i); // right lanes

        if (window.labelOverlays) {
            window.labelOverlays.forEach(o => o.setMap(null));
        }
        window.labelOverlays = [];
        for (let i of laneIndices) {
            // Compute center offset for this lane
            const offsetDistance = laneWidth * i;
            const offsetX = -px * offsetDistance;
            const offsetY = -py * offsetDistance;

            // Rectangle center offset from center line
            const cx1 = x1 + offsetX;
            const cy1 = y1 + offsetY;
            const cx2 = x2 + offsetX;
            const cy2 = y2 + offsetY;

            // Compute corners
            const halfWidthX = px * (laneWidth / 2);
            const halfWidthY = py * (laneWidth / 2);

            const corners = [
                { x: cx1 + halfWidthX, y: cy1 + halfWidthY },
                { x: cx1 - halfWidthX, y: cy1 - halfWidthY },
                { x: cx2 - halfWidthX, y: cy2 - halfWidthY },
                { x: cx2 + halfWidthX, y: cy2 + halfWidthY }
            ];

            const path = corners.map(({ x, y }) => {
                const lat = toDegrees(y / R);
                const lng = toDegrees(x / (R * Math.cos((lat1 + lat2) / 2)));
                return { lat, lng };
            });

            const polygon = new google.maps.Polygon({
                paths: path,
                map: map,
                strokeColor: i === laneIndices[0] || i === laneIndices[laneIndices.length - 1]  ? '#32302F' : '#0000FF',
                strokeOpacity: 0.8,
                strokeWeight: 2,
                fillColor: i === laneIndices[0] || i === laneIndices[laneIndices.length - 1] ? '#32302F' : '#0000FF',
                fillOpacity: 0.2,
                zIndex: i === laneIndices[0] || i === laneIndices[laneIndices.length - 1] ? 1 : 2
            });

            window.rectangleOverlays.push(polygon);

            let angleDeg = -Math.atan2(dy, dx) * 180 / Math.PI;
            if (angleDeg < -90 || angleDeg > 90) angleDeg += 180;

            const midX = (cx1 + cx2) / 2;
            const midY = (cy1 + cy2) / 2;
            const midLat = toDegrees(midY / R);
            const midLng = toDegrees(midX / (R * Math.cos((lat1 + lat2) / 2)));
            const midpoint = new google.maps.LatLng(midLat, midLng);

            // Draw speed advisory on outermost lanes
            if (i === laneIndices[0] || i === laneIndices[laneIndices.length - 1]) {
                const advisoryOverlay = new SpeedAdvisoryTextOverlay(midpoint, angleDeg);
                advisoryOverlay.setMap(map);
                window.labelOverlays.push(advisoryOverlay);
            } else {
                const closedOverlay = new ClosedTextOverlay(midpoint, angleDeg);
                closedOverlay.setMap(map);
                window.labelOverlays.push(closedOverlay);
            }
        }
    }
}

function getLatLngFromPoint(point, map) {
    const scale = Math.pow(2, map.getZoom());
    const projection = map.getProjection();

    if (!projection) {
        console.error("Map projection not ready.");
        return map.getCenter();
    }

    const bounds = map.getBounds();
    const nw = new google.maps.LatLng(
        bounds.getNorthEast().lat(),
        bounds.getSouthWest().lng()
    );

    const worldCoordinateNW = projection.fromLatLngToPoint(nw);
    const pixelOffset = new google.maps.Point(point.x / scale, point.y / scale);
    const worldPoint = new google.maps.Point(
        worldCoordinateNW.x + pixelOffset.x,
        worldCoordinateNW.y + pixelOffset.y
    );

    return projection.fromPointToLatLng(worldPoint);
}

function getMarkerIcon(type) {
    switch (type) {
        case 'start-zone':
            return 'http://www.google.com/mapfiles/markerS.png';
        case 'end-zone':
            return 'http://www.google.com/mapfiles/markerE.png';
        default:
            return null;
    }
}

// Function to trigger map resize when container becomes visible
function resizeMap() {
    if (map && mapInitialized) {
        google.maps.event.trigger(map, 'resize');
        if (hostmarker) {
            map.setCenter(hostmarker.getPosition());
        }
    }
}

function getHighlightedIcon(type) {
    const base = getMarkerIcon(type);
    return {
        url: base,
        scaledSize: new google.maps.Size(50, 50),  // Highlighted = larger icon
        anchor: new google.maps.Point(25, 50)
    };
}

// Load API key from environment file
async function loadApiKey() {
    try {
        const apiKey = await getApiKey();
        if (!apiKey || apiKey === 'ERROR_API_KEY') {
            throw new Error('Invalid or missing Google Maps API key');
        }
        console.log('Google Maps API key loaded successfully');
        return apiKey;
    } catch (error) {
        console.error('Failed to load API key:', error);
        throw error;
    }
}

// Get API key from configuration
async function getApiKey() {
    try {
        const response = await fetch('/google_map_api_key.env');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }

        const content = await response.text();
        const lines = content.split('\n');

        for (const line of lines) {
            const trimmed = line.trim();

            if (!trimmed || trimmed.startsWith('#') || !trimmed.includes('=')) {
                continue;
            }

            const [key, ...valueParts] = trimmed.split('=');

            if (key.trim() === 'GOOGLE_MAPS_API_KEY' && valueParts.length > 0) {
                let apiKey = valueParts.join('=').trim();

                // Remove surrounding quotes if present
                if ((apiKey.startsWith('"') && apiKey.endsWith('"')) ||
                    (apiKey.startsWith("'") && apiKey.endsWith("'"))) {
                    apiKey = apiKey.slice(1, -1);
                }

                console.log('API key loaded from env file');
                return apiKey;
            }
        }
    } catch (error) {
        console.warn('Could not fetch env file via HTTP:', error.message);
    }

    console.error('GOOGLE_MAPS_API_KEY not found in any accessible location');
    return 'ERROR_API_KEY';
}

/*
    Draws the route on the map.
*/
function setRouteMap(map) {
    if (!map) return;

    // Get the saved route
    const savedRoute = sessionStorage.getItem('routePlanCoordinates');
    if (!savedRoute) {
        return;
    }

    try {
        routePlanCoordinates = JSON.parse(savedRoute);

        // Maps the selected route on the map
        const routePath = new google.maps.Polyline({
            path: routePlanCoordinates,
            geodesic: true,
            strokeColor: '#6495ed', // cornflowerblue
            strokeOpacity: 1.0,
            strokeWeight: 2
        });

        routePath.setMap(map);
    } catch (error) {
        console.error('Error parsing route coordinates:', error);
    }
}

/*
    Maps the initial position of the host vehicle.
*/
function setHostMarker() {
    if (!map) {
        console.error('Map not initialized when trying to set host marker');
        return;
    }

    var VehicleLatValue = $('#LatitudeSpan').text();
    var VehicleLonValue = $('#LongitudeSpan').text();
    var LatRange = { MIN: -90, MAX: 90 };
    var LonRange = { MIN: -180, MAX: 180 };
    if(VehicleLatValue == "" || isNaN(VehicleLatValue) || VehicleLatValue > LatRange.MAX || VehicleLatValue < LatRange.MIN) {
        alert("Vehicle latitude is required, should be between " + LatRange.MIN + " and " + LatRange.MAX);
        return false;
    }
    if(VehicleLonValue == "" || isNaN(VehicleLonValue) || VehicleLonValue > LonRange.MAX || VehicleLonValue < LonRange.MIN) {
        alert("Vehicle longitude is required, should be between " + LonRange.MIN + " and " + LonRange.MAX);
        return false;
    }
    var vehicleLat = parseFloat(VehicleLatValue);
    var vehicleLon = parseFloat(VehicleLonValue);

    // Add host vehicle marker
    const marker = new google.maps.Marker({
        id: 'mHostVehicle',
        position: { lat: vehicleLat, lng: vehicleLon },
        map: map,
        icon: 'http://www.google.com/mapfiles/ms/icons/blue-dot.png',
        title: 'Host Vehicle',
        zIndex: 1
    });

    hostmarker = marker;
    window.hostmarker = marker;
    map.setCenter(hostmarker.getPosition());
}

/*
    To paint the pin a particular color.
*/
function pinSymbol(color) {
    return {
        path: 'M 0,0 C -2,-20 -10,-22 -10,-30 A 10,10 0 1,1 10,-30 C 10,-22 2,-20 0,0 z',
        fillColor: color,
        fillOpacity: 1,
        strokeColor: '#000',
        strokeWeight: 2,
        scale: 1
    };
}

/*
    Maps other CAV vehicles on the map.
*/
function setOtherVehicleMarkers(id, latitude, longitude) {
    if (!map || !mapInitialized) {
        console.warn('Map not ready for setting vehicle markers');
        return;
    }

    if (!markers) {
        markers = [];
    }

    const targetMarker = findMarker(markers, id);
    if (!targetMarker) {
        // Vehicle ID is not on the list of markers. Add to the map.
        addMarkerForOtherVehicleWithTimeout(id, latitude, longitude, 100);
    } else {
        // Vehicle ID has been found. Update the location of the marker.
        moveMarkerWithTimeout(targetMarker, latitude, longitude, 100);
    }

    // Update markers in session storage
    sessionStorage.setItem('mapMarkers', JSON.stringify(markers));
}

/*
    Move a marker to a new position.
*/
function moveMarkerWithTimeout(myMarker, newLat, newLong, timeout) {
    setTimeout(function() {
        if (myMarker && map && google && google.maps) {
            myMarker.setPosition(new google.maps.LatLng(newLat, newLong));

            if (myMarker.id === 'mHostVehicle') {
                // Center map based on the host vehicle marker only if not using start marker location
                if (!useStartMarkerLocation) {
                    map.setCenter(myMarker.getPosition());
                }
            }
        }
    }, timeout);
}

/*
 * Draw a polygon on the map based on list of geo positions
 */
function drawPolygonsOnMap(polygon_type, vector_Geo_locations) {
    if (!Array.isArray(vector_Geo_locations) || vector_Geo_locations.length !== 4) {
        console.error("vector_Geo_locations for polygon is not an array or does not contain 4 geo-locations.");
        return;
    }

    if (!map || !mapInitialized) {
        console.error('Map not initialized for drawing polygons');
        return;
    }

    if (polygon_type === g_polygon_type.TCR) {
        // Remove existing TCR polygon first
        if (tcr_polygon) {
            tcr_polygon.setMap(null);
            tcr_polygon = null;
        }

        // Create content string for info window
        const contentStr = "<b>TCR bounding box</b><br> <li>Drawing polygon from position3D list in a clockwise direction.</li>" +
            "<li>Position3D list</li> - latitude:" + vector_Geo_locations[0].lat + ", longitude:" + vector_Geo_locations[0].lng + " <br>" +
            "- latitude:" + vector_Geo_locations[1].lat + ", longitude:" + vector_Geo_locations[1].lng + " <br>" +
            "- latitude:" + vector_Geo_locations[2].lat + ", longitude:" + vector_Geo_locations[2].lng + " <br>" +
            "- latitude:" + vector_Geo_locations[3].lat + ", longitude:" + vector_Geo_locations[3].lng + " <br>";

        // TCR request bounding box on the google map
        tcr_polygon = new google.maps.Polygon({
            paths: vector_Geo_locations,
            strokeColor: "#000000",
            strokeOpacity: 1,
            strokeWeight: 2,
            fillColor: "#FFA500",
            fillOpacity: 0.3
        });

        const infoWindow = new google.maps.InfoWindow();

        tcr_polygon.setMap(map);
        window.tcr_polygon = tcr_polygon;

        tcr_polygon.addListener("click", (event) => {
            infoWindow.setContent(contentStr);
            infoWindow.setPosition(event.latLng);
            infoWindow.open(map);
        });
    }
}

/*
    Find a marker by ID.
*/
function findMarker(allMarkers, idToFind) {
    if (!allMarkers) return null;

    for (let i = 0; i < allMarkers.length; i++) {
        if (allMarkers[i].id === idToFind) {
            return allMarkers[i];
        }
    }
    return null;
}

/*
    Add a marker for other vehicles.
*/
function addMarkerForOtherVehicleWithTimeout(newId, newLat, newLong, timeout) {
    setTimeout(function() {
        if (!markers) {
            markers = [];
        }

        if (map && google && google.maps) {
            const marker = new google.maps.Marker({
                id: newId,
                position: new google.maps.LatLng(newLat, newLong),
                map: map,
                title: newId,
                dateTimeCreated: new Date(),
            });

            markers.push(marker);
            window.markers = markers;
        }
    }, timeout);
}

/*
    Delete a marker by ID.
*/
function deleteMarker(id) {
    if (!markers) return;

    for (let i = markers.length - 1; i >= 0; i--) {
        if (markers[i].id === id) {
            // Remove the marker from Map
            markers[i].setMap(null);
            // Remove the marker from array
            markers.splice(i, 1);
            break;
        }
    }
    window.markers = markers;
}

// Cleanup function to stop GPS broadcasting when page unloads
window.addEventListener('beforeunload', function() {
    stopGPSBroadcasting();
});

// Make functions globally available
window.initializeMapWhenReady = initializeMapWhenReady;
window.resizeMap = resizeMap;
window.drawPolygonsOnMap = drawPolygonsOnMap;
window.setOtherVehicleMarkers = setOtherVehicleMarkers;
window.moveMarkerWithTimeout = moveMarkerWithTimeout;
window.findMarker = findMarker;
window.deleteMarker = deleteMarker;
window.useStartMarkerLocation = () => useStartMarkerLocation;
window.startGPSBroadcasting = startGPSBroadcasting;
window.stopGPSBroadcasting = stopGPSBroadcasting;
window.getCurrentLocationAndUpdateHost = getCurrentLocationAndUpdateHost;
window.updateHostMarkerToCurrentLocation = updateHostMarkerToCurrentLocation;
