const express = require('express');
const net = require('net');
const path = require('path');

const app = express();
const PORT = process.env.PORT || 3000;
const RELAY_HOST = '124.56.147.40';
const RELAY_PORT = 7700;

// ── Relay Protocol ──────────────────────────────────────────────────────────
const RELAY_MAGIC = Buffer.from([0x42, 0x52, 0x4C, 0x59]); // 'BRLY'
const PKT_LIST_REQ  = 0x20;
const PKT_LIST_RESP = 0x21;
const HDR_SIZE = 9; // 4(magic) + 1(type) + 4(len)

// RelayStation: 32(id) + 64(name) + 4(lat) + 4(lon) + 1(tier) + 1(users) + 2(pad) = 108
// RelayListResp header: 2(count) + 1(lan_ip_count) + 8*16(lan_ips) = 131
const STATION_SIZE = 108;
const LIST_RESP_HDR = 131;

let cachedStations = [];

function fetchStations() {
    return new Promise((resolve) => {
        const sock = new net.Socket();
        sock.setTimeout(3000);
        const chunks = [];

        sock.connect(RELAY_PORT, RELAY_HOST, () => {
            // Send LIST_REQ
            const pkt = Buffer.alloc(HDR_SIZE);
            RELAY_MAGIC.copy(pkt, 0);
            pkt[4] = PKT_LIST_REQ;
            pkt.writeUInt32LE(0, 5); // payload len = 0
            sock.write(pkt);
        });

        sock.on('data', (data) => chunks.push(data));

        sock.on('end', () => {
            try {
                const buf = Buffer.concat(chunks);
                if (buf.length < HDR_SIZE) return resolve([]);
                // Verify magic
                if (buf[0]!==0x42||buf[1]!==0x52||buf[2]!==0x4C||buf[3]!==0x59) return resolve([]);
                if (buf[4] !== PKT_LIST_RESP) return resolve([]);
                const payloadLen = buf.readUInt32LE(5);
                const payload = buf.subarray(HDR_SIZE, HDR_SIZE + payloadLen);
                if (payload.length < LIST_RESP_HDR) return resolve([]);

                const count = payload.readUInt16LE(0);
                const stations = [];

                for (let i = 0; i < count; i++) {
                    const off = LIST_RESP_HDR + i * STATION_SIZE;
                    if (off + STATION_SIZE > payload.length) break;

                    const idBuf = payload.subarray(off, off + 32);
                    const nameBuf = payload.subarray(off + 32, off + 96);
                    const lat = payload.readFloatLE(off + 96);
                    const lon = payload.readFloatLE(off + 100);
                    const tier = payload[off + 104];
                    const users = payload[off + 105];

                    stations.push({
                        station_id: idBuf.toString('utf8').replace(/\0/g, ''),
                        name: nameBuf.toString('utf8').replace(/\0/g, ''),
                        lat, lon, tier, users
                    });
                }
                resolve(stations);
            } catch (e) {
                console.error('Parse error:', e.message);
                resolve([]);
            }
        });

        sock.on('error', () => resolve([]));
        sock.on('timeout', () => { sock.destroy(); resolve([]); });
    });
}

// Poll relay every 5 seconds
async function pollLoop() {
    while (true) {
        try {
            cachedStations = await fetchStations();
        } catch (e) { /* ignore */ }
        await new Promise(r => setTimeout(r, 5000));
    }
}
pollLoop();

// ── Express ─────────────────────────────────────────────────────────────────
app.use(express.static(path.join(__dirname, 'public')));

app.get('/api/stations', (req, res) => {
    res.json(cachedStations);
});

app.listen(PORT, () => {
    console.log(`BEWE Web running on http://localhost:${PORT}`);
});
