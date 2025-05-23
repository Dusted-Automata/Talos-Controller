let path = L.layerGroup();
const points = [];
const ecef_points = [];
let Renderer = L.canvas({padding: 0.5});

function toRadians(degrees) {
    return degrees * Math.PI / 180;
}

function latLngToECEF(lat, lng, alt = 0) {
    // WGS84 ellipsoid constants
    const a = 6378137.0; // semi-major axis in meters
    const e = 0.081819190842622; // first eccentricity
    
    // Convert latitude and longitude to radians
    const latRad = toRadians(lat);
    const lngRad = toRadians(lng);
    
    // Calculate N, the radius of curvature in the prime vertical
    const N = a / Math.sqrt(1 - Math.pow(e * Math.sin(latRad), 2));
    
    // Calculate ECEF coordinates
    const x = (N + alt) * Math.cos(latRad) * Math.cos(lngRad);
    const y = (N + alt) * Math.cos(latRad) * Math.sin(lngRad);
    const z = (N * (1 - Math.pow(e, 2)) + alt) * Math.sin(latRad);
    
    return [x, y, z];
}

function saveECEFPointsAsJSON() {
    if (ecef_points.length === 0) {
        alert("No points to export!");
        return;
    }
    
    const pointsData = {
        type: "ECEFPoints",
        count: ecef_points.length,
        points: ecef_points.map((point, index) => ({
            id: index,
            x: point[0],
            y: point[1],
            z: point[2],
            lat: points[index][0],
            lon: points[index][1],
            alt: points[index][2]
        }))
    };
    
    const jsonData = JSON.stringify(pointsData, null, 2);
    const blob = new Blob([jsonData], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "ecef_points.json";
    
    document.body.appendChild(a);
    a.click();
    URL.revokeObjectURL(url);
    document.body.removeChild(a);
}

const tiles = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
	maxNativeZoom: 19,
	maxZoom: 25,
	attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
});

const map = L.map('map', {
    center: [49.767663, 6.627878],
    zoom: 18,
    zoomDelta: 0.25,
    zoomSnap: 0,
    layers: [tiles],
    contextmenu: true,
    contextmenuWidth: 140,
    contextmenuItems: [{
        text: 'Export ECEF Points as JSON',
        callback: saveECEFPointsAsJSON
    }]
});

const popup = L.popup();

async function getElevation(lat, lng) {
    const apiUrl = `https://api.open-elevation.com/api/v1/lookup?locations=${lat},${lng}`;
    const res = await fetch(apiUrl);
    if (!res.ok) {
        throw new Error('Network response was not ok');
    }
    
    const data = await res.json();
    return data.results[0].elevation;
}

async function onMapClick(e) {
    const alt = await getElevation(e.latlng.lat, e.latlng.lng);
    console.log('Retrieved Elevation:', alt);
    points.push([e.latlng.lat, e.latlng.lng, alt]);
    ecef_points.push(latLngToECEF(e.latlng.lat, e.latlng.lng, alt));
    
    updatePolyline();
    
    const marker = L.marker([e.latlng.lat, e.latlng.lng], {
        draggable: true,
        pointIndex: points.length - 1
    }).addTo(path);
    
    marker.on('dragend', async function(e) {
        const marker = e.target;
        const position = marker.getLatLng();
        const pointIndex = marker.options.pointIndex;
        
        const newAlt = await getElevation(position.lat, position.lng);
        
        points[pointIndex] = [position.lat, position.lng, newAlt];
        ecef_points[pointIndex] = latLngToECEF(position.lat, position.lng, newAlt);
        
        updatePolyline();
        
        console.log(`Point ${pointIndex} moved to:`, points[pointIndex]);
        console.log(`Updated ECEF:`, ecef_points[pointIndex]);
    });
    
    if (!path.pointMarkers) {
        path.pointMarkers = [];
    }
    path.pointMarkers.push(marker);
    
    let formattedPoints = ecef_points.map(point => `{${point[0]}, ${point[1]}, ${point[2]}}`);
    console.log(`{${formattedPoints.join(', ')}}`);
}

function updatePolyline() {
    path.eachLayer(layer => {
        if (layer instanceof L.Polyline && !(layer instanceof L.Marker)) {
            path.removeLayer(layer);
        }
    });
    
    if (points.length > 0) {
        const latLngs = points.map(point => [point[0], point[1]]);
        L.polyline(latLngs, { color: "red", renderer: Renderer }).addTo(path);
    }
}

function onKeyPress(e) {
    const key = e.originalEvent.key;
    
    if (key === 'd'){
        points.pop();
        ecef_points.pop();
        
        if (path.pointMarkers && path.pointMarkers.length > 0) {
            const lastMarker = path.pointMarkers.pop();
            path.removeLayer(lastMarker);
        }
        
        updatePolyline();
    }
    
    if (key === 'e') {
        saveECEFPointsAsJSON();
    }
}

L.Control.ExportButton = L.Control.extend({
    options: {
        position: 'topright'
    },
    
    onAdd: function(map) {
        const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
        const button = L.DomUtil.create('a', 'export-button', container);
        
        button.innerHTML = 'Export Points';
        button.href = '#';
        button.title = 'Export ECEF Points as JSON';
        
        L.DomEvent.on(button, 'click', L.DomEvent.stop)
                 .on(button, 'click', saveECEFPointsAsJSON);
        
        return container;
    }
});

L.control.exportButton = function(opts) {
    return new L.Control.ExportButton(opts);
};

L.control.exportButton().addTo(map);

map.addLayer(path);
map.on('click', onMapClick);
map.on('keypress', onKeyPress);

map.on('contextmenu', function(e) {
    L.popup()
        .setLatLng(e.latlng)
        .setContent('<button class="export-json-btn">Export ECEF Points as JSON</button>')
        .openOn(map);
    
    document.querySelector('.export-json-btn').addEventListener('click', function() {
        saveECEFPointsAsJSON();
        map.closePopup();
    });
});
