
let path = L.layerGroup();
const points = []

let Renderer = L.canvas({padding: 0.5});


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
	points.push([e.latlng.lat, e.latlng.lng])
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
