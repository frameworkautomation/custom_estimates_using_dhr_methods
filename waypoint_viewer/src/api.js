const BASE = '/api'

export async function getWaypoints() {
  const r = await fetch(`${BASE}/waypoints`)
  if (!r.ok) throw new Error(`GET /api/waypoints failed: ${r.status}`)
  return r.json()
}

export async function getEdges() {
  const r = await fetch(`${BASE}/edges`)
  if (!r.ok) throw new Error(`GET /api/edges failed: ${r.status}`)
  return r.json()
}

export async function createEdge(from_name, to_name) {
  const r = await fetch(`${BASE}/edges`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ from_name, to_name }),
  })
  if (!r.ok) throw new Error(`POST /api/edges failed: ${r.status}`)
  return r.json()
}

export async function deleteEdge(from_name, to_name) {
  const r = await fetch(`${BASE}/edges`, {
    method: 'DELETE',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ from_name, to_name }),
  })
  if (!r.ok) throw new Error(`DELETE /api/edges failed: ${r.status}`)
  return r.json()
}

export async function getGuiSettings() {
  const r = await fetch(`${BASE}/gui_settings`)
  if (!r.ok) throw new Error(`GET /api/gui_settings failed: ${r.status}`)
  return r.json()
}

export async function getOrigins() {
  const r = await fetch(`${BASE}/origins`)
  if (!r.ok) throw new Error(`GET /api/origins failed: ${r.status}`)
  return r.json()
}
