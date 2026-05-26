import React from 'react'

const STATUS_COLORS = { 'true': '#44bb44', 'false': '#ff4444', 'null': '#888' }

// A single labelled field row with an optional grey description
function Row({ label, value, desc }) {
  if (value == null) return null
  return (
    <div style={{ display: 'flex', gap: '6px', marginTop: '3px', alignItems: 'baseline' }}>
      <span style={{ color: '#a0c4ff', flexShrink: 0 }}>{label}:</span>
      <span style={{ color: '#fff' }}>{String(value)}</span>
      {desc && <span style={{ color: '#666', fontSize: '10px' }}>— {desc}</span>}
    </div>
  )
}

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
          {[
            ['x', wp.x?.toFixed(1), 'TCP position X (mm)'],
            ['y', wp.y?.toFixed(1), 'TCP position Y (mm)'],
            ['z', wp.z?.toFixed(1), 'TCP position Z (mm)'],
            ['rx', wp.rx?.toFixed(2), 'Rotation about X (deg, ZYX Euler)'],
            ['ry', wp.ry?.toFixed(2), 'Rotation about Y (deg, ZYX Euler)'],
            ['rz', wp.rz?.toFixed(2), 'Rotation about Z (deg, ZYX Euler)'],
          ].filter(([, v]) => v != null).map(([k, v, desc]) => (
            <Row key={k} label={k} value={v} desc={desc} />
          ))}
          <Row
            label="frame"
            value={wp.frame}
            desc={
              wp.frame === 'world'       ? 'Absolute RoboDK world coordinates' :
              wp.frame === 'robot_local' ? 'Relative to robot base at j7=0 (rail home)' :
              null
            }
          />
          <Row
            label="source"
            value={wp.source}
            desc={
              wp.source === 'human'       ? 'Captured via save_joint_position.py' :
              wp.source === 'grasshopper' ? 'Exported by Grasshopper script' :
              null
            }
          />
          {wp.tool_name != null && (
            <Row label="tool" value={wp.tool_name ?? 'null'} desc="Active RoboDK tool when recorded" />
          )}
          {wp.joints && (
            <Row
              label="joints"
              value={`[${wp.joints.map(j => Number(j).toFixed(1)).join(', ')}]`}
              desc="Joint angles (deg), j7 in mm"
            />
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
