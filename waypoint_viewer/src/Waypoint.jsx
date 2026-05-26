import React, { useMemo } from 'react'
import * as THREE from 'three'
import { Line } from '@react-three/drei'

const WAYPOINT_COLOR = '#44bb44'

// Compute all 3 axis directions from ZYX Euler angles (degrees).
// R = Rz * Ry * Rx. Remaps robot→scene coords: [robot_x, robot_z, robot_y].
function frameAxes(rxDeg, ryDeg, rzDeg) {
  const toRad = (d) => (d * Math.PI) / 180
  const rx = toRad(rxDeg ?? 0)
  const ry = toRad(ryDeg ?? 0)
  const rz = toRad(rzDeg ?? 0)
  const cx = Math.cos(rx), sx = Math.sin(rx)
  const cy = Math.cos(ry), sy = Math.sin(ry)
  const cz = Math.cos(rz), sz = Math.sin(rz)

  // Columns of R = Rz*Ry*Rx in robot space.
  // Scene mapping: scene = [-robot_x, robot_z, robot_y]
  // Negating X restores right-handedness after the Y↔Z axis swap.
  const xAxis = new THREE.Vector3(-(cz * cy),               -sy,          sz * cy)
  const yAxis = new THREE.Vector3(-(cz * sy * sx - sz * cx), cy * sx,  sz * sy * sx + cz * cx)
  const zAxis = new THREE.Vector3(-(cz * sy * cx + sz * sx), cy * cx,  sz * sy * cx - cz * sx)

  return { xAxis, yAxis, zAxis }
}

const SPHERE_R = 25  // mm

export default function Waypoint({ waypoint: wp, isSelected, onSelect, axisLength }) {
  const ARROW_LEN = axisLength ?? 37.5
  // Scene mapping: [-robot_x, robot_z, robot_y]
  const pos = [-(wp.x ?? 0), wp.z ?? 0, wp.y ?? 0]
  const sphereColor = isSelected ? '#ffff00' : WAYPOINT_COLOR

  const { xAxis, yAxis, zAxis } = useMemo(
    () => frameAxes(wp.rx, wp.ry, wp.rz),
    [wp.rx, wp.ry, wp.rz]
  )

  const tip = (axis) => [
    pos[0] + axis.x * ARROW_LEN,
    pos[1] + axis.y * ARROW_LEN,
    pos[2] + axis.z * ARROW_LEN,
  ]

  return (
    <group>
      <mesh
        position={pos}
        onClick={(e) => { e.stopPropagation(); onSelect(wp.name) }}
      >
        <sphereGeometry args={[SPHERE_R, 16, 16]} />
        <meshStandardMaterial color={sphereColor} />
      </mesh>

      {/* XYZ trihedron — red/green/blue matching RoboDK convention */}
      <Line points={[pos, tip(xAxis)]} color="#ff3333" lineWidth={2} />
      <Line points={[pos, tip(yAxis)]} color="#33cc33" lineWidth={2} />
      <Line points={[pos, tip(zAxis)]} color="#3388ff" lineWidth={2} />
    </group>
  )
}
