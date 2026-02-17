#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

import trimesh
import trimesh.creation
import numpy as np
import os
from shapely.geometry import LineString, Polygon

class WaamSlicerViz(Node):
    def __init__(self):
        super().__init__('waam_slicer_viz')
        self.marker_pub = self.create_publisher(MarkerArray, 'waam_visualizacao', 10)
        
        # --- CONFIGURAÇÕES ---
        self.stl_path = "/workspaces/xyzRobotROS/src/Funil_v1.stl"
        self.layer_height = 3.5   # mm
        self.bead_width = 1.0     # mm
        
        # Controle da Animação
        self.animation_speed = 0.1 # Segundos entre cada camada (quanto menor, mais rápido)
        self.layers_buffer = []    # Guarda todas as camadas calculadas
        self.current_layer_idx = 0 # Qual camada estamos mostrando agora
        self.published_markers = [] # Acumula os markers para o RViz
        
        # Começa o processamento
        self.calculate_layers()
        
        # Inicia o Timer da Animação
        self.timer = self.create_timer(self.animation_speed, self.animation_callback)

    def calculate_layers(self):
        self.get_logger().info(f"1. Carregando e processando geometria...")
        
        # --- CARREGAMENTO ---
        if os.path.exists(self.stl_path):
            mesh = trimesh.load(self.stl_path)

            rot_y = trimesh.transformations.rotation_matrix(np.pi/2, [1,0,0])
            mesh.apply_transform(rot_y)
            
            # === NOVA PARTE: ESCALA ===
            # Fator 2.0 = Dobra o tamanho
            # Fator 1000.0 = Converte Metros para Milimetros (Muito util!)
            # Fator 25.4 = Converte Polegadas para Milimetros
            scale_factor = 1.0
            mesh.apply_scale(scale_factor)
            
            self.get_logger().info(f"Escala de {scale_factor}x aplicada!")
            # ==========================
            
        else:
            self.get_logger().warn("Arquivo não encontrado! Criando cubo de teste...")
            mesh = trimesh.creation.box(extents=[50, 50, 50])

        # Centraliza e põe no chão
        mesh.apply_translation(-mesh.centroid)
        min_z = mesh.bounds[0][2]
        mesh.apply_translation([0, 0, -min_z])
        
        max_z = mesh.bounds[1][2]
        z_levels = np.arange(self.layer_height, max_z, self.layer_height)
        
        id_counter = 0

        # --- LOOP DE FATIAMENTO (CÁLCULO APENAS) ---
        self.get_logger().info(f"Calculando {len(z_levels)} camadas...")
        
        for z in z_levels:
            current_layer_markers = [] # Lista de markers apenas desta camada
            
            slice_3d = mesh.section(plane_origin=[0,0,z], plane_normal=[0,0,1])
            if slice_3d is None: continue

            slice_2d, _ = slice_3d.to_2D()
            polygons = slice_2d.polygons_full

            for poly in polygons:
                # --- LÓGICA DE PEÇA SÓLIDA (PREENCHIMENTO) ---
                # 1. Primeiro contorno (parede externa)
                current_poly = poly.buffer(-self.bead_width / 2.0)
                pass_count = 0
                
                while not current_poly.is_empty:
                    pass_count += 1
                    if current_poly.geom_type == 'MultiPolygon':
                        geoms = current_poly.geoms
                    else:
                        geoms = [current_poly]

                    for geom in geoms:
                        coords = list(geom.exterior.coords)
                        
                        marker = Marker()
                        marker.header.frame_id = "map"
                        marker.type = Marker.LINE_STRIP
                        marker.action = Marker.ADD
                        marker.id = id_counter
                        id_counter += 1
                        marker.scale.x = 1.0 
                        
                        # Gradiente de Cor (Azul -> Vermelho)
                        intensity = z / max_z
                        marker.color.r = intensity
                        marker.color.g = (pass_count * 0.2) % 1.0 
                        marker.color.b = 1.0 - intensity
                        marker.color.a = 1.0

                        for x, y in coords:
                            p = Point()
                            p.x, p.y, p.z = x, y, z
                            marker.points.append(p)
                        
                        current_layer_markers.append(marker)
                    
                    # Recua para fazer o anel interno
                    current_poly = current_poly.buffer(-self.bead_width)
            
            # Guarda os markers dessa camada no buffer global
            if current_layer_markers:
                self.layers_buffer.append(current_layer_markers)

        self.get_logger().info(f"Cálculo pronto! Iniciando animação com {len(self.layers_buffer)} quadros.")

    def animation_callback(self):
        # Se ainda tem camadas para mostrar
        if self.current_layer_idx < len(self.layers_buffer):
            
            # Pega os novos markers da camada atual
            new_markers = self.layers_buffer[self.current_layer_idx]
            
            # Adiciona na lista "visível"
            self.published_markers.extend(new_markers)
            
            # Publica tudo o que já foi acumulado
            msg = MarkerArray()
            msg.markers = self.published_markers
            self.marker_pub.publish(msg)
            
            # Avança o índice
            self.current_layer_idx += 1
            
            # Log discreto a cada 10 camadas
            if self.current_layer_idx % 10 == 0:
                self.get_logger().info(f"Animando camada {self.current_layer_idx}...")

        else:
            # CHEGOU NO FINAL: Reinicia a animação
            self.get_logger().info("Fim da peça! Reiniciando em 3 segundos...")
            
            # Para limpar a tela do RViz, mandamos um Marker com ação DELETEALL
            delete_msg = MarkerArray()
            marker_del = Marker()
            marker_del.action = Marker.DELETEALL
            delete_msg.markers.append(marker_del)
            self.marker_pub.publish(delete_msg)
            
            # Reseta as variáveis
            self.published_markers = []
            self.current_layer_idx = 0
            
            # Pequeno delay (gambiarra visual: dorme o timer)
            # Na próxima chamada ele começa do zero

def main(args=None):
    rclpy.init(args=args)
    node = WaamSlicerViz()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()