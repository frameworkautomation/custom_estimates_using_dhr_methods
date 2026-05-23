import React from 'react'

const STATUS_COLORS = { 'true': '#44bb44', 'false': '#ff4444', 'null': '#888' }

function statusStr(tested) {
  return tested === null ? 'null' : String(tested)
}

export default function InfoPanel({ selected, edges, onCreateEdge, onDeleteEdge }) {
  // Edges where at least one endpoint is a selected waypoint
  const relevantEdges = edges.filter(e =>
    selected.some(wp => e.from === wp.name || e.to === wp.name)
  )

  return (
    <div style={{
      width: '300px',
      background: '#16213e',
      color: '#e0e0e0',
      padding: '16px',
      overflowY: 'auto',
      fontFamily: 'monospace',
      fontSize: '12px',
      flexShrink: 0,
    }}>
      <h3 style={{ margin: '0 0 12px', color: '#a0c4ff' }}>
        Selection ({selected.length}/2)
      </h3>

      {selected.length === 0 && (
        <p style={{ color: '#555' }}>Click waypoints to select. Select 2 to create an edge.</p>
      )}

      {selected.map(wp => (
        <div key={wp.name} style={{
          marginBottom: '10px', padding: '8px',
          background: '#0f3460', borderRadius: '4px',
        }}>
          <div style={{ fontWeight: 'bold', color: '#ffff00', marginBottom: '4px' }}>
            {wp.name}
          </div>
          {'x' in wp && (
            <div style={{ color: '#ccc' }}>
              <div>x: {wp.x?.toFixed(1)}  y: {wp.y?.toFixed(1)}  z: {wp.z?.toFixed(1)}</div>
              <div>rx: {wp.rx?.toFixed(1)}  ry: {wp.ry?.toFixed(1)}  rz: {wp.rz?.toFixed(1)}</div>
            </div>
          )}
          <div style={{ marginTop: '4px', color: '#aaa' }}>
            {wp.move_type && <span style={{ marginRight: '8px' }}>{wp.move_type}</span>}
            {wp.frame && <span style={{ marginRight: '8px' }}>{wp.frame}</span>}
            {wp.source && <span>{wp.source}</span>}
          </div>
          {wp.joints && (
            <div style={{ color: '#888', marginTop: '4px' }}>
              joints: [{wp.joints.map(j => Number(j).toFixed(1)).join(', ')}]
            </div>
          )}
        </div>
      ))}

      {selected.length === 2 && (
        <button
          onClick={onCreateEdge}
          style={{
            width: '100%', padding: '10px', marginBottom: '16px',
            background: '#2a6a2a', color: '#7fff7f',
            border: '1px solid #44bb44', borderRadius: '4px',
            cursor: 'pointer', fontFamily: 'monospace', fontWeight: 'bold',
          }}
        >
          + Create Bidirectional Edge
        </button>
      )}

      {relevantEdges.length > 0 && (
        <>
          <h3 style={{ margin: '8px 0 8px', color: '#a0c4ff' }}>Connected Edges</h3>
          {relevantEdges.map((e, i) => {
            const s = statusStr(e.tested)
            return (
              <div key={i} style={{
                display: 'flex', alignItems: 'center',
                marginBottom: '4px', gap: '6px',
              }}>
                <span style={{
                  flex: 1, color: STATUS_COLORS[s] ?? '#888',
                  overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap',
                }}>
                  {e.from} &rarr; {e.to}
                </span>
                <span style={{ color: STATUS_COLORS[s] ?? '#888', fontSize: '10px', flexShrink: 0 }}>
                  {s}
                </span>
                <button
                  onClick={() => onDeleteEdge(e.from, e.to)}
                  style={{
                    background: '#4a1a1a', color: '#ff8888',
                    border: '1px solid #ff4444', borderRadius: '3px',
                    cursor: 'pointer', padding: '1px 6px', fontSize: '11px', flexShrink: 0,
                  }}
                >
                  &times;
                </button>
              </div>
            )
          })}
        </>
      )}
    </div>
  )
}
