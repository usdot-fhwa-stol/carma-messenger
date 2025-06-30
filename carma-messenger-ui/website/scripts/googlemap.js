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
 This file shall contain Map related functions - Fixed version
****/

// Global variables
let map;
let markers = [];
let hostmarker;
let tcr_polygon;
let routePlanCoordinates;
let mapInitialized = false;
let placedMarkers = [];
let closedLabelOverlay = null;

// Polygon types
const g_polygon_type = {
    TCR: 'TCR'
};

$(document).ready(() => {
    // Don't initialize map immediately, wait for container to be ready
    console.log('Document ready, waiting for map container...');
});

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
        script.src = `https://maps.googleapis.com/maps/api/js?key=${apiKey}&loading=async&callback=showNewMap`;
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

    document.getElementById('StartLat').addEventListener('input', updateStartZoneMarkerFromForm);
    document.getElementById('StartLon').addEventListener('input', updateStartZoneMarkerFromForm);
    document.getElementById('EndLat').addEventListener('input', updateEndZoneMarkerFromForm);
    document.getElementById('EndLon').addEventListener('input', updateEndZoneMarkerFromForm);

    mapDiv.addEventListener('drop', (e) => {
        e.preventDefault();

        const markerType = e.dataTransfer.getData("text/plain");
        const mapBounds = map.getDiv().getBoundingClientRect();

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
    
        // Convert start and end to Cartesian coordinates (meters)
        const x1 = R * lng1 * Math.cos((lat1 + lat2) / 2);
        const y1 = R * lat1;
        const x2 = R * lng2 * Math.cos((lat1 + lat2) / 2);
        const y2 = R * lat2;
    
        // Direction vector from start to end
        const dx = x2 - x1;
        const dy = y2 - y1;
        const length = Math.sqrt(dx * dx + dy * dy);
    
        // Perpendicular unit vector
        const px = -dy / length;
        const py = dx / length;
    
        // Half-width offset (2 meters)
        const offsetX = px * 2;
        const offsetY = py * 2;
    
        // Compute 4 corners
        const corners = [
            { x: x1 + offsetX, y: y1 + offsetY },
            { x: x1 - offsetX, y: y1 - offsetY },
            { x: x2 - offsetX, y: y2 - offsetY },
            { x: x2 + offsetX, y: y2 + offsetY }
        ];
    
        // Convert back to LatLng
        const path = corners.map(({ x, y }) => {
            const lat = toDegrees(y / R);
            const lng = toDegrees(x / (R * Math.cos((lat1 + lat2) / 2)));
            return { lat, lng };
        });
    
        // Remove old rectangle if it exists
        if (rectangleOverlay) {
            rectangleOverlay.setMap(null);
        }
    
        rectangleOverlay = new google.maps.Polygon({
            paths: path,
            map: map,
            strokeColor: '#0000FF',
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: '#0000FF',
            fillOpacity: 0.1
        });

        // Remove old label if it exists
        if (closedLabelOverlay) {
            closedLabelOverlay.setMap(null);
        }

        // Midpoint
        const midLat = (start.lat() + end.lat()) / 2;
        const midLng = (start.lng() + end.lng()) / 2;
        const midpoint = new google.maps.LatLng(midLat, midLng);

        // Angle of line (in degrees)
        // Get angle from start to end in degrees
        let angleDeg = -Math.atan2(dy, dx) * 180 / Math.PI;

        // Normalize angle to keep text upright
        if (angleDeg < -90 || angleDeg > 90) {
            angleDeg += 180;
        }

        // Create label overlay
        closedLabelOverlay = new ClosedTextOverlay(midpoint, angleDeg);
        closedLabelOverlay.setMap(map);
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
        case 'end-zone':
            return 'http://www.google.com/mapfiles/markerE.png';
        case 'start-zone':
            return 'http://www.google.com/mapfiles/markerS.png';
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
                // Center map based on the host vehicle marker
                map.setCenter(myMarker.getPosition());
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

// Make functions globally available
window.initializeMapWhenReady = initializeMapWhenReady;
window.resizeMap = resizeMap;
window.drawPolygonsOnMap = drawPolygonsOnMap;
window.setOtherVehicleMarkers = setOtherVehicleMarkers;
window.moveMarkerWithTimeout = moveMarkerWithTimeout;
window.findMarker = findMarker;
window.deleteMarker = deleteMarker;
