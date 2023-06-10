mapboxgl.accessToken = 'pk.eyJ1IjoiamFheWFudGgiLCJhIjoiY2xpcHVpMHlzMG01MDNmbGI1NTZoNDVpciJ9.GO9DSZT5E9tGdEKg8WAp9Q';

// Initialize the map
var map = new mapboxgl.Map({
    container: 'map',
    style: 'mapbox://styles/mapbox/streets-v11',
    center: [80.23021913617647, 12.993036661158213], // Default center coordinates (San Francisco)
    zoom: 15
});

// Array of nearby coordinates
var coordinates = [
    [80.23221913617647, 12.997036661158213],
    [80.23521913617647, 12.998036661158213],
    [80.23121913617647, 12.993036661158213],
    [80.23621913617647, 12.992036661158213],
    [80.23721913617647, 12.991036661158213],
    [80.23321913617647, 12.998036661158213],
    [80.23521913617647, 12.993036661158213],
    [80.23521913617647, 12.999036661158213],
    [80.23721913617647, 12.996036661158213],
    [80.23121913617647, 12.999036661158213],
    [80.23821913617647, 12.991036661158213],
    [80.23221913617647, 12.992036661158213],
    [80.23921913617647, 12.998036661158213],
    [80.23221913617647, 12.999036661158213],
    [80.23721913617647, 12.994036661158213],
    [80.23321913617647, 12.990036661158213],
    [80.23921913617647, 12.993036661158213],
    [80.23021913617647, 12.992036661158213],
    [80.23221913617647, 12.998036661158213],
    [80.23121913617647, 12.998036661158213],
];

// Create a GeoJSON feature collection for the path
var path = {
    'type': 'FeatureCollection',
    'features': [{
        'type': 'Feature',
        'geometry': {
            'type': 'LineString',
            'coordinates': []
        }
    }]
};

// Create a custom marker
var marker = new mapboxgl.Marker({
        element: createMarkerElement(),
        rotationAlignment: 'map',
        draggable: false
    })
    .setLngLat(coordinates[0])
    .addTo(map);

var currentIndex = 0;

// Move marker to the next coordinate
function moveMarker() {
    var currentCoord = coordinates[currentIndex];
    marker.setLngLat(currentCoord);

    // Add the current coordinate to the path
    path.features[0].geometry.coordinates.push(currentCoord);

    currentIndex++;
    if (currentIndex >= coordinates.length) {
        currentIndex = 0;
    }

    // Update the map's data source with the new path
    map.getSource('path').setData(path);

    // Recenter the map to the marker's location
    map.setCenter(currentCoord);
}

// Create a custom marker element
function createMarkerElement() {
    var img = document.createElement('img');
    img.src = 'location_marker.png'; // Replace 'marker.png' with your marker image file name
    img.style.width = '30px';
    img.style.height = '30px';
    return img;
}

// Update marker rotation based on movement direction
function updateMarkerRotation() {
    if (currentIndex < coordinates.length - 1) {
        var currentCoord = coordinates[currentIndex];
        var nextCoord = coordinates[currentIndex + 1];

        var rotation = calculateBearing(currentCoord, nextCoord);
        marker.setRotation(rotation);
    }
}

// Calculate bearing (angle) between two coordinates
function calculateBearing(startCoord, endCoord) {
    var startLat = degToRad(startCoord[1]);
    var startLng = degToRad(startCoord[0]);
    var endLat = degToRad(endCoord[1]);
    var endLng = degToRad(endCoord[0]);

    var dLng = endLng - startLng;

    var y = Math.sin(dLng) * Math.cos(endLat);
    var x =
        Math.cos(startLat) * Math.sin(endLat) -
        Math.sin(startLat) * Math.cos(endLat) * Math.cos(dLng);

    var bearing = radToDeg(Math.atan2(y, x));
    return (bearing + 360) % 360;
}

// Convert degrees to radians
function degToRad(degrees) {
    return degrees * (Math.PI / 180);
}

// Convert radians to degrees
function radToDeg(radians) {
    return radians * (180 / Math.PI);
}

// Move the marker every second
setInterval(function () {
    moveMarker();
    updateMarkerRotation();
}, 1000);

// Add the path layer to the map
map.on('load', function () {
    map.addSource('path', {
        type: 'geojson',
        data: path
    });

    map.addLayer({
        id: 'path',
        type: 'line',
        source: 'path',
        paint: {
            'line-color': '#4287f5',
            'line-opacity': 0.8,
            'line-width': 3
        }
    });
});