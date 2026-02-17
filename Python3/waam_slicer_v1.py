#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import trimesh
import numpy as np
import os
from shapely.geometry import LineString, Polygon

class WaamSlicerViz(Node):
    def __init__(self):
        super().__init__('waam_slicer_viz')
        self.marker_pub = self.create_publisher(MarkerArray, 'waam_visualizacao', 10)
        
        # --- CONFIGURAÇÕES DO USUÁRIO ---
        # Caminho do STL (Pode ser absoluto ou relativo)
        # Exemplo: "/workspaces/xyzRobotROS/src/minha_peca.stl"
        self.stl_path = "/workspaces/xyzRobotROS/src/Funil_v1.stl" 
        
        self.layer_height = 1.5   # mm
        self.bead_width = 4.0     # mm
        
        self.process_and_publish()

    def process_and_publish(self):
        self.get_logger().info(f"1. Carregando arquivo: {self.stl_path}")
        
        if os.path.exists(self.stl_path):
            mesh = trimesh.load(self.stl_path)
            
            # === NOVA PARTE: ESCALA ===
            # Fator 2.0 = Dobra o tamanho
            # Fator 1000.0 = Converte Metros para Milimetros (Muito util!)
            # Fator 25.4 = Converte Polegadas para Milimetros
            scale_factor = 250
            mesh.apply_scale(scale_factor)
            
            self.get_logger().info(f"Escala de {scale_factor}x aplicada!")
            # ==========================
            
        else:
            self.get_logger().warn("Arquivo não encontrado! Criando cubo de teste...")
            mesh = trimesh.creation.box(extents=[50, 50, 50])

        # --- PREPARAÇÃO DA MALHA (CRUCIAL PARA WAAM) ---
        # 1. Centraliza X e Y (Centro da peça vai para 0,0)
        # 2. Coloca a base em Z=0
        mesh.apply_translation(-mesh.centroid) # Move centro para 0,0,0
        # Agora ajusta só o Z para a base tocar o chão
        min_z = mesh.bounds[0][2]
        mesh.apply_translation([0, 0, -min_z])
        
        self.get_logger().info("2. Fatiando malha...")
        
        # Descobre a altura total da peça dinamicamente
        max_z = mesh.bounds[1][2]
        z_levels = np.arange(self.layer_height, max_z, self.layer_height)
        
        marker_array = MarkerArray()
        id_counter = 0

        for z in z_levels:
            # Fatiamento 3D -> 2D
            slice_3d = mesh.section(plane_origin=[0,0,z], plane_normal=[0,0,1])
            if slice_3d is None: continue

            slice_2d, _ = slice_3d.to_2D()
            polygons = slice_2d.polygons_full

            for poly in polygons:
                # Offset (Compensação do Raio)
                inset_poly = poly.buffer(-self.bead_width / 2.0)

                if inset_poly.is_empty: continue

                # Trata MultiPoligonos (Ex: peça com dois furos ou duas torres)
                if inset_poly.geom_type == 'MultiPolygon':
                    geoms = inset_poly.geoms
                else:
                    geoms = [inset_poly]

                for geom in geoms:
                    coords = list(geom.exterior.coords)
                    
                    # --- CRIAÇÃO DO VISUALIZADOR ---
                    marker = Marker()
                    marker.header.frame_id = "map"
                    marker.type = Marker.LINE_STRIP
                    marker.action = Marker.ADD
                    marker.id = id_counter
                    id_counter += 1
                    marker.scale.x = 1.0 
                    
                    # Cor baseada na altura
                    intensity = z / max_z
                    marker.color.r = intensity
                    marker.color.g = 1.0 - intensity # Verde para Vermelho
                    marker.color.b = 0.2
                    marker.color.a = 1.0

                    for x, y in coords:
                        p = Point()
                        p.x, p.y, p.z = x, y, z
                        marker.points.append(p)
                    
                    marker_array.markers.append(marker)

        self.get_logger().info(f"3. SUCESSO! {len(marker_array.markers)} camadas geradas.")
        self.timer = self.create_timer(1.0, lambda: self.marker_pub.publish(marker_array))

def main(args=None):
    rclpy.init(args=args)
    node = WaamSlicerViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()