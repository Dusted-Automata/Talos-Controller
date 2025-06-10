let path = L.layerGroup();
const points = [];
let Renderer = L.canvas({padding: 0.5});

function toRadians(degrees) {
    return degrees * Math.PI / 180;
}

function savePointsAsJson() {
    if (points.length === 0) {
        alert("No points to export!");
        return;
    }
    
    const pointsData = {
        type: "LatLngPoints",
        count: points.length,
        points: points.map((point, index) => ({
            id: index,
            lat: point[0],
            lon: point[1],
            alt: point[2]
        }))
    };
    
    const jsonData = JSON.stringify(pointsData, null, 2);
    const blob = new Blob([jsonData], { type: "application/json" });
    const url = URL.createObjectURL(blob);
    const a = document.createElement("a");
    a.href = url;
    a.download = "LatLng_Points.json";
    
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
        text: 'Export Lat Lng Points as JSON',
        callback: savePointsAsJson
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
        
        updatePolyline();
        
        console.log(`Point ${pointIndex} moved to:`, points[pointIndex]);
    });
    
    if (!path.pointMarkers) {
        path.pointMarkers = [];
    }
    path.pointMarkers.push(marker);
    
    let formattedPoints = points.map(point => `{${point[0]}, ${point[1]}, ${point[2]}}`);
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
        
        if (path.pointMarkers && path.pointMarkers.length > 0) {
            const lastMarker = path.pointMarkers.pop();
            path.removeLayer(lastMarker);
        }
        
        updatePolyline();
    }
    
    if (key === 'e') {
        savePointsAsJson();
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
        button.title = 'Export LatLng Points as JSON';
        
        L.DomEvent.on(button, 'click', L.DomEvent.stop)
                 .on(button, 'click', savePointsAsJson);
        
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
        .setContent('<button class="export-json-btn">Export LatLng Points as JSON</button>')
        .openOn(map);
    
    document.querySelector('.export-json-btn').addEventListener('click', function() {
        savePointsAsJson();
        map.closePopup();
    });
});
