
let path = L.layerGroup();
const points = []
const ecef_points = []

let Renderer = L.canvas({padding: 0.5});

function latLngToECEF(lat, lng, alt = 0) {
    // WGS84 ellipsoid constants
    const a = 6378137.0; // semi-major axis in meters
    const e = 0.081819190842622; // first eccentricity
    
    // Convert latitude and longitude to radians
    const latRad = lat * (Math.PI / 180.0);
    const lngRad = lng * (Math.PI / 180.0);
    
    // Calculate N, the radius of curvature in the prime vertical
    const N = a / Math.sqrt(1 - Math.pow(e * Math.sin(latRad), 2));
    
    // Calculate ECEF coordinates
    const x = (N + alt) * Math.cos(latRad) * Math.cos(lngRad);
    const y = (N + alt) * Math.cos(latRad) * Math.sin(lngRad);
    const z = (N * (1 - Math.pow(e, 2)) + alt) * Math.sin(latRad);
    
    return [ x, y, z ];
}


const tiles = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png', {
	maxZoom: 19,
	attribution: '&copy; <a href="http://www.openstreetmap.org/copyright">OpenStreetMap</a>'
});

const map = L.map('map', {
	center: [49.767663, 6.627878],
	zoom: 18,
	layers: [tiles]
	// renderer: L.canvas()
});

const popup = L.popup();

function onMapClick(e) {
	points.push([e.latlng.lat, e.latlng.lng]);
	ecef_points.push(latLngToECEF(e.latlng.lat, e.latlng.lng, 241));
	// console.log(map.hasLayer(path))
	// map.removeLayer(path);
	let polyline = L.polyline(points, {color: "red", renderer: Renderer}).addTo(path)
	// map.addLayer(path);

	// let polyline = L.polyline(points, {color: "red"}).addTo(map)
	console.log(points)
}

function onKeyPress(e) {
	key = e.originalEvent.key
	if (key === 'j'){
		map.removeLayer(path);
		console.log(map);
	}
	if (key === 'd'){
		map.removeLayer(path);
		points.pop()
		path = L.layerGroup()
		map.addLayer(path);
	}


}


map.addLayer(path);
map.on('click', onMapClick);
map.on('keypress', onKeyPress);
