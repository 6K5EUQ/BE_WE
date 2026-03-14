// ── BEWE Globe Web ──────────────────────────────────────────────────────────
const canvas = document.getElementById('globe');
const tooltip = document.getElementById('tooltip');
const popup = document.getElementById('popup');
const stationCountEl = document.getElementById('station-count');

// ── Three.js Setup ──────────────────────────────────────────────────────────
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, alpha: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x000000, 1);

const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 0.1, 100);
camera.position.set(0, 0, 3.5);

window.addEventListener('resize', () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});

// ── Stars ───────────────────────────────────────────────────────────────────
(function createStars() {
    const geo = new THREE.BufferGeometry();
    const n = 3000;
    const pos = new Float32Array(n * 3);
    for (let i = 0; i < n; i++) {
        const theta = Math.random() * Math.PI * 2;
        const phi = Math.acos(2 * Math.random() - 1);
        const r = 30 + Math.random() * 20;
        pos[i*3]   = r * Math.sin(phi) * Math.cos(theta);
        pos[i*3+1] = r * Math.sin(phi) * Math.sin(theta);
        pos[i*3+2] = r * Math.cos(phi);
    }
    geo.setAttribute('position', new THREE.BufferAttribute(pos, 3));
    const mat = new THREE.PointsMaterial({ color: 0xffffff, size: 0.05, sizeAttenuation: true });
    scene.add(new THREE.Points(geo, mat));
})();

// ── Globe ───────────────────────────────────────────────────────────────────
const GLOBE_RADIUS = 1.0;
const globeGeo = new THREE.SphereGeometry(GLOBE_RADIUS, 64, 48);

// Custom shader for rim glow
const globeMat = new THREE.ShaderMaterial({
    uniforms: {
        uTexture: { value: null },
        hasTexture: { value: 0.0 }
    },
    vertexShader: `
        varying vec2 vUv;
        varying vec3 vNormal;
        varying vec3 vPosition;
        void main() {
            vUv = uv;
            vNormal = normalize(normalMatrix * normal);
            vPosition = (modelViewMatrix * vec4(position, 1.0)).xyz;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
    `,
    fragmentShader: `
        uniform sampler2D uTexture;
        uniform float hasTexture;
        varying vec2 vUv;
        varying vec3 vNormal;
        varying vec3 vPosition;
        void main() {
            vec3 viewDir = normalize(-vPosition);
            float rim = 1.0 - max(dot(viewDir, vNormal), 0.0);
            rim = pow(rim, 2.5);
            vec3 baseColor;
            if (hasTexture > 0.5) {
                baseColor = texture2D(uTexture, vUv).rgb;
                // Darken slightly for cinematic feel
                baseColor *= 0.85;
            } else {
                baseColor = vec3(0.04, 0.10, 0.28);
            }
            vec3 rimColor = vec3(0.3, 0.5, 0.9);
            vec3 final = baseColor + rimColor * rim * 0.5;
            gl_FragColor = vec4(final, 1.0);
        }
    `
});

const globe = new THREE.Mesh(globeGeo, globeMat);
scene.add(globe);

// Atmosphere glow
const atmosGeo = new THREE.SphereGeometry(GLOBE_RADIUS * 1.03, 64, 48);
const atmosMat = new THREE.ShaderMaterial({
    uniforms: {},
    vertexShader: `
        varying vec3 vNormal;
        varying vec3 vPosition;
        void main() {
            vNormal = normalize(normalMatrix * normal);
            vPosition = (modelViewMatrix * vec4(position, 1.0)).xyz;
            gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
        }
    `,
    fragmentShader: `
        varying vec3 vNormal;
        varying vec3 vPosition;
        void main() {
            vec3 viewDir = normalize(-vPosition);
            float intensity = pow(1.0 - max(dot(viewDir, vNormal), 0.0), 3.0);
            gl_FragColor = vec4(0.3, 0.5, 1.0, intensity * 0.35);
        }
    `,
    transparent: true,
    side: THREE.BackSide,
    depthWrite: false
});
scene.add(new THREE.Mesh(atmosGeo, atmosMat));

// Load earth texture
const texLoader = new THREE.TextureLoader();
texLoader.load('assets/earth.jpg', (tex) => {
    tex.anisotropy = renderer.capabilities.getMaxAnisotropy();
    globeMat.uniforms.uTexture.value = tex;
    globeMat.uniforms.hasTexture.value = 1.0;
});

// ── Station Markers ─────────────────────────────────────────────────────────
let stations = [];
const markerGroup = new THREE.Group();
scene.add(markerGroup);

function latLonToVec3(lat, lon, r) {
    const phi = (90 - lat) * Math.PI / 180;
    const theta = (lon + 180) * Math.PI / 180;
    return new THREE.Vector3(
        -r * Math.sin(phi) * Math.cos(theta),
         r * Math.cos(phi),
         r * Math.sin(phi) * Math.sin(theta)
    );
}

function createMarkerSprite() {
    const size = 64;
    const cv = document.createElement('canvas');
    cv.width = size; cv.height = size;
    const ctx = cv.getContext('2d');

    // Outer glow
    const grad = ctx.createRadialGradient(size/2, size/2, 0, size/2, size/2, size/2);
    grad.addColorStop(0,   'rgba(100,180,255,0.9)');
    grad.addColorStop(0.25,'rgba(60,140,255,0.6)');
    grad.addColorStop(0.5, 'rgba(40,100,220,0.2)');
    grad.addColorStop(1,   'rgba(20,60,160,0.0)');
    ctx.fillStyle = grad;
    ctx.fillRect(0, 0, size, size);

    // Core
    ctx.beginPath();
    ctx.arc(size/2, size/2, 5, 0, Math.PI * 2);
    ctx.fillStyle = '#fff';
    ctx.fill();

    const tex = new THREE.CanvasTexture(cv);
    return tex;
}

const markerTex = createMarkerSprite();

function updateMarkers() {
    // Remove old markers
    while (markerGroup.children.length) markerGroup.remove(markerGroup.children[0]);

    stations.forEach((st, i) => {
        const pos = latLonToVec3(st.lat, st.lon, GLOBE_RADIUS * 1.005);
        const mat = new THREE.SpriteMaterial({
            map: markerTex,
            transparent: true,
            depthTest: false,
            blending: THREE.AdditiveBlending
        });
        const sprite = new THREE.Sprite(mat);
        sprite.position.copy(pos);
        sprite.scale.set(0.08, 0.08, 1);
        sprite.userData = { stationIdx: i };
        markerGroup.add(sprite);
    });
}

// ── Pulse animation for markers ─────────────────────────────────────────────
let pulseTime = 0;
function animateMarkers(dt) {
    pulseTime += dt;
    const pulse = 0.08 + Math.sin(pulseTime * 2.5) * 0.015;
    markerGroup.children.forEach(s => s.scale.set(pulse, pulse, 1));
}

// ── Mouse Interaction ───────────────────────────────────────────────────────
let isDragging = false;
let prevMouse = { x: 0, y: 0 };
let targetRotX = -0.18; // Initial view: slightly tilted to show Korea
let targetRotY = -2.22; // ~127E longitude
let rotX = targetRotX;
let rotY = targetRotY;
let zoomDist = 3.5;
let targetZoom = 3.5;

canvas.addEventListener('mousedown', (e) => {
    if (e.button === 0) {
        isDragging = true;
        prevMouse = { x: e.clientX, y: e.clientY };
    }
});

window.addEventListener('mouseup', () => { isDragging = false; });

window.addEventListener('mousemove', (e) => {
    if (isDragging) {
        const dx = e.clientX - prevMouse.x;
        const dy = e.clientY - prevMouse.y;
        // Scale drag speed by zoom
        const scale = zoomDist / 3.5 * 0.005;
        targetRotY += dx * scale;
        targetRotX += dy * scale;
        // Clamp vertical rotation
        targetRotX = Math.max(-Math.PI/2 + 0.1, Math.min(Math.PI/2 - 0.1, targetRotX));
        prevMouse = { x: e.clientX, y: e.clientY };
    }
    handleHover(e);
});

canvas.addEventListener('wheel', (e) => {
    e.preventDefault();
    targetZoom += e.deltaY * 0.002;
    targetZoom = Math.max(1.5, Math.min(8.0, targetZoom));
}, { passive: false });

canvas.addEventListener('click', (e) => {
    handleClick(e);
});

// ── Hover / Click Detection ─────────────────────────────────────────────────
const raycaster = new THREE.Raycaster();
const mouse2d = new THREE.Vector2();

function getHoveredStation(e) {
    mouse2d.x = (e.clientX / window.innerWidth) * 2 - 1;
    mouse2d.y = -(e.clientY / window.innerHeight) * 2 + 1;
    raycaster.setFromCamera(mouse2d, camera);

    // Check distance to each marker in screen space
    let bestDist = 25; // pixel threshold
    let bestIdx = -1;

    stations.forEach((st, i) => {
        const pos = latLonToVec3(st.lat, st.lon, GLOBE_RADIUS * 1.005);
        // Apply globe rotation
        pos.applyEuler(new THREE.Euler(rotX, rotY, 0, 'YXZ'));
        const projected = pos.clone().project(camera);
        const sx = (projected.x * 0.5 + 0.5) * window.innerWidth;
        const sy = (-projected.y * 0.5 + 0.5) * window.innerHeight;
        const dist = Math.hypot(e.clientX - sx, e.clientY - sy);

        // Check if marker is on visible side (not behind globe)
        const camDir = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
        const toMarker = pos.clone().sub(camera.position).normalize();
        if (projected.z > 1) return; // Behind camera

        if (dist < bestDist) {
            bestDist = dist;
            bestIdx = i;
        }
    });
    return bestIdx;
}

function handleHover(e) {
    if (popup.classList.contains('hidden') === false) return;
    const idx = getHoveredStation(e);
    if (idx >= 0) {
        const st = stations[idx];
        const tierLabel = st.tier === 1 ? 'Tier 1' : 'Tier 2';
        tooltip.innerHTML = `${st.name} <span class="tier-tag">${tierLabel}</span>`;
        tooltip.style.left = (e.clientX + 16) + 'px';
        tooltip.style.top = (e.clientY - 10) + 'px';
        tooltip.classList.remove('hidden');
        canvas.style.cursor = 'pointer';
    } else {
        tooltip.classList.add('hidden');
        canvas.style.cursor = isDragging ? 'grabbing' : 'grab';
    }
}

function handleClick(e) {
    if (isDragging) return;
    const idx = getHoveredStation(e);
    if (idx >= 0) {
        showPopup(stations[idx]);
    } else if (!popup.classList.contains('hidden')) {
        closePopup();
    }
}

function showPopup(st) {
    document.getElementById('popup-name').textContent = st.name;

    const latDir = st.lat >= 0 ? 'N' : 'S';
    const lonDir = st.lon >= 0 ? 'E' : 'W';
    document.getElementById('popup-coords').textContent =
        `${Math.abs(st.lat).toFixed(4)}°${latDir}  ${Math.abs(st.lon).toFixed(4)}°${lonDir}`;

    document.getElementById('popup-users').textContent =
        `${st.users} Operator(s) connected`;

    const tierEl = document.getElementById('popup-tier');
    tierEl.textContent = st.tier === 1 ? 'Tier 1' : 'Tier 2';
    tierEl.className = 'popup-tier ' + (st.tier === 1 ? 'tier1' : 'tier2');

    popup.classList.remove('hidden');
    tooltip.classList.add('hidden');
}

function closePopup() {
    popup.classList.add('hidden');
}

// ── Station Polling ─────────────────────────────────────────────────────────
async function pollStations() {
    try {
        const res = await fetch('/api/stations');
        if (res.ok) {
            stations = await res.json();
            updateMarkers();
            stationCountEl.textContent = stations.length === 0
                ? 'No Stations Online'
                : `${stations.length} Station${stations.length > 1 ? 's' : ''} Online`;
        }
    } catch (e) { /* silent */ }
}
pollStations();
setInterval(pollStations, 1000);

// ── Render Loop ─────────────────────────────────────────────────────────────
let lastTime = performance.now();

function animate() {
    requestAnimationFrame(animate);
    const now = performance.now();
    const dt = (now - lastTime) / 1000;
    lastTime = now;

    // Smooth rotation
    rotX += (targetRotX - rotX) * 0.08;
    rotY += (targetRotY - rotY) * 0.08;
    zoomDist += (targetZoom - zoomDist) * 0.08;

    // Auto-rotate when not dragging
    if (!isDragging && popup.classList.contains('hidden')) {
        targetRotY += dt * 0.03;
    }

    // Apply rotation to globe + markers
    globe.rotation.set(rotX, rotY, 0, 'YXZ');
    markerGroup.rotation.set(rotX, rotY, 0, 'YXZ');

    // Camera zoom
    camera.position.z = zoomDist;

    // Animate marker pulse
    animateMarkers(dt);

    renderer.render(scene, camera);
}
animate();
