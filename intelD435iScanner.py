import time
import cv2  # Package zum Erstellen und Speichern der Bilder
import numpy as np  # Package für mathematische Berechnungen und Datenspeicherung der Frames
import pyrealsense2 as rs  # Realsense Package für die Kamera
import open3d as o3d  # Open3D Package für die Verarbeitungspipeline nach der Aufnahme

class Scan():
    def __init__(self, pNumberScans: int, pSize: str, pStand: str):
        """
        :param pNumberScans: Anzahl der Aufnahmen, welche vom Objekt gemacht werden sollen. Der Wert darf entweder zwei
        oder vier sein.
        :param pSize: Größe des Objektes, was aufgenommen werden soll. Es gibt eine Unterscheidung zwischen b für big,
        also größere Objekte und c für coin, also Münzen oder andere kleinere Objekte
        :param pStand: Variable, ob das Objekt auf einem Standfuß steht. y für Ja und n für Nein
        """

        # Standardwerte = Werte die im Intel RealSense Viewer nativ so vorgesehen sind
        self.size = pSize  # Initialisierung von size mit dem Parameter pSize
        self.stand = pStand  # Initialisierung von stand mit dem Parameter pStand
        self.numberScans = pNumberScans
        self.apr = 360/self.numberScans


        self.width = 848  # Initialisierung von width, also der Breite des Scans, mit dem Standardwert
        self.height = 480  # Initialisierung von height, also der Höhe des Scans, mit dem Standartwert
        self.framerate = 30  # Initialisierung von framerate, also der Framerate des Streams (bzw. der Aufnahme mit dem Standardwert)
        self.drc = np.pi / 180  # Bogenmaß=Gradzahl*drc (degree to radial conversion)
        self.complete_pcd = o3d.geometry.PointCloud()  # Initialisierung der leeren PointCloud => Später zusammenfügen der einzelnen PointClouds

        self.pipe = rs.pipeline()  # Initialisieren der Realsense Pipeline zur Aufnahme der Bilder
        self.config = rs.config()  # Initialisieren der Realsense Config zur Pipeline zur Aufnahme der Bilder
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.any,
                                  self.framerate)  # Konfigurierung des Farbstreams
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.any,
                                  self.framerate)  # Konfigurierung des Tiefenstreams

        #self.distance = 0.1822 #0.1795 neu 0.1814 FÜR FRANKFURT
        #self.distance = 0.1650 # FÜR HEIMAT
        #self.distance = 0.1613 # Frankfurt halbe Ständer
        # self.distance = 0.1578 # Münze
        if self.size == 'b':
            self.distance = 0.17
        elif self.size == 'c':
            self.distance = 0.1578
        else:
            self.distance = 0.16

        """
            Details für Bounding Box:
            Für die Bounding Box Abstände zum Objekt: ca 176 bis 213
            Bei Tiefenkamera Objekt zwischen: Von links 205, rechts 245, oben 123, unten 158
            Bei RGB Kamera Objekt zwischen: Von linksseitig 675, rechtsseitig 870, oben 390, unten 586
        """
        if self.size == 'b' and self.stand == 'n':
            self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.0625), (0.2, 0.02, 0.11))  # Herz Bounding Box
            self.bbox2 = o3d.geometry.AxisAlignedBoundingBox((-0.003, -0.07, 0.06), (0.05, 0.2, 0.11))
            self.bbox3 = o3d.geometry.AxisAlignedBoundingBox((-0.1, -0.1, 0.0626), (0.1, 0.1, 0.11))
            self.bbox4 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.06), (0.02, 0.1, 0.11))
        elif self.size == 'b' and self.stand == 'y':
            self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.079), (0.2, 0.02, 0.11))  # Größe der Box, welche nicht abgeschnitten wird
            self.bbox2 = o3d.geometry.AxisAlignedBoundingBox((-0.003, -0.07, 0.07), (0.05, 0.2, 0.11))
            self.bbox3 = o3d.geometry.AxisAlignedBoundingBox((-0.1, -0.1, 0.08), (0.1, 0.1, 0.11))
            self.bbox4 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.07), (0.02, 0.1, 0.11))
        elif self.size == 'c' and self.stand == 'y':
            print("Ich bin hier!")
            self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.079), (
            0.02, 0.02, 0.11))  # Größe der Box, welche nicht abgeschnitten wird
            self.bbox2 = o3d.geometry.AxisAlignedBoundingBox((-0.003, -0.07, 0.07), (0.05, 0.2, 0.11))
            self.bbox3 = o3d.geometry.AxisAlignedBoundingBox((-0.1, -0.1, 0.08), (0.1, 0.1, 0.11))
            self.bbox4 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.07), (0.02, 0.1, 0.11))
        elif self.size == 'c' and self.stand == 'n':
            self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.0625), (
            0.02, 0.02, 0.11))  # Größe der Box, welche nicht abgeschnitten wird
            self.bbox2 = o3d.geometry.AxisAlignedBoundingBox((-0.003, -0.07, 0.06), (0.05, 0.2, 0.11))
            self.bbox3 = o3d.geometry.AxisAlignedBoundingBox((-0.1, -0.1, 0.0626), (0.1, 0.1, 0.11))
            self.bbox4 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.06), (0.02, 0.1, 0.11))
        else:
            self.bbox1 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.079), (
                0.02, 0.02, 0.11))  # Größe der Box, welche nicht abgeschnitten wird
            self.bbox2 = o3d.geometry.AxisAlignedBoundingBox((-0.003, -0.07, 0.07), (0.05, 0.2, 0.11))
            self.bbox3 = o3d.geometry.AxisAlignedBoundingBox((-0.1, -0.1, 0.08), (0.1, 0.1, 0.11))
            self.bbox4 = o3d.geometry.AxisAlignedBoundingBox((-0.048, -0.048, 0.07), (0.02, 0.1, 0.11))


    def startPipeline(self):
        """Startet die Streaming Pipeline, sodass Arbeit mit der Kamera möglich.
        :return Falls die Kamera angeschlossen ist (und somit die Pipeline gestartet werden kann), wird 1 zurückgegeben
        Ist dies nicht der Fall, wird 0 zurückgegeben. Dies verhindert im Verlauf Fehler durch unnötige Aufrufe.
        """
        try:
            self.pipe.start(self.config)
            self.align = rs.align(rs.stream.color)
            return 1
        except:
            print("Kein Gerät angeschlossen!")
            return 0


    def stopPipeline(self):
        """Stoppt die Streaming Pipeline, sodass kein Stream mehr läuft. Genutzt zur Beendigung des Prozesses"""
        self.pipe.stop()
        self.pipe = None
        self.config = None

    def takeFoto(self):
        """Macht die Aufnahmen mit der Kamera. Also sowohl die Tiefenaufnahme, als auch ein RGB Bild."""

        for i in range(10):
            self.frameset = self.pipe.wait_for_frames() # Aufnahme des Bildes zur Verbesserung
            # der Qualität (vor allem Helligkeit) => Idee ist Auto Exposure (Intelligente, automatische Belichtung)

        # Foto aufnehmen
        self.frameset = self.pipe.wait_for_frames() # Aufnahme des Objektes
        self.frameset = self.align.process(self.frameset) # Aligning der vorhandenen Streams
        self.profile = self.frameset.get_profile() # Holt die Meta Informationen des Streams
        self.depth_intrinsics = self.profile.as_video_stream_profile().get_intrinsics()
        self.w, self.h = self.depth_intrinsics.width, self.depth_intrinsics.height
        self.fx, self.fy = self.depth_intrinsics.fx, self.depth_intrinsics.fy
        self.px, self.py = self.depth_intrinsics.ppx, self.depth_intrinsics.ppy

        self.color_frame = self.frameset.get_color_frame()
        self.depth_frame = self.frameset.get_depth_frame()

        self.intrinsic = o3d.camera.PinholeCameraIntrinsic(self.w, self.h, self.fx, self.fy, self.px, self.py)
        self.depth_image = np.asanyarray(self.depth_frame.get_data())
        self.color_image = np.asanyarray(self.color_frame.get_data())

    def processFoto(self, pAngle: int):
        '''
        Die Aufnahme, welche mit takeFoto gemacht wurde, wird nun weiterverarbeitet und unter anderem zur PointCloud
        hinzugefügt.
        :param pAngle (int): Der Winkel aus welchem das Foto aufgenommen wurde, relativ zum Objekt.
        '''
        print("Aktueller Aufnahmewinkel: " + str(pAngle))
        self.angle = pAngle
        self.depth_frame_open3d = o3d.geometry.Image(self.depth_image)  # Erstellung Tiefenaufnahme als Bild
        self.color_frame_open3d = o3d.geometry.Image(self.color_image)  # Erstellung Farbaufnahme als Bild

        self.rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(self.color_frame_open3d,
                                                                             self.depth_frame_open3d,
                                                                             convert_rgb_to_intensity=False)  # Zusammensetzen des Gesamtbildes (RGBD) aus Farb- und Tiefenbild
        self.pcd = o3d.geometry.PointCloud.create_from_rgbd_image(self.rgbd_image,
                                                                  self.intrinsic)  # Erstelle PointCloud aus RGBD Bild und Intrinsics

        self.pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))  # Bestimmung Normalvektoren
        self.pcd.orient_normals_towards_camera_location(
            camera_location=np.array([0., 0., 0.]))  # Ausrichtung der Normalvektoren hinsichtlich der Kameraposition
        self.getObjectRotation()
        self.correctRotation()
        self.pcd.rotate(self.rotMatrix, (0, 0, 0))  # PointCloud rotieren
        self.pcd.translate((self.x, self.y, self.z))  # PointCloud anpassen

        if self.angle == 0:
            self.pcd = self.pcd.crop(self.bbox1)
        elif self.angle == 90:
            self.pcd = self.pcd.crop(self.bbox2)
        elif self.angle == 180:
            self.pcd = self.pcd.crop(self.bbox3)
        elif self.angle == 270 :
            self.pcd = self.pcd.crop(self.bbox4)
        else:
            print("Fehler beim Abschneiden der Bounding Box!")
            pass

        self.pcd, self.ind = self.pcd.remove_statistical_outlier(nb_neighbors=100,
                                                                 std_ratio=2)  # Entfernung weiterer Ausreisser
        o3d.visualization.draw_geometries([self.pcd])
        self.complete_pcd = self.complete_pcd + self.pcd  # Zusammensetzen der PointCloud

    def getPointcloud(self):
        '''
        Liefert die aktuelle Point Cloud.
        :return: Die aktuelle Point Cloud.
        '''
        return self.complete_pcd

    def getImageArray(self):
        '''
        Liefert das aktuelle RGB Bild als Array.
        :return: Ein Array, welches das aktuelle RGB Bild darstellt.
        '''
        return self.color_image

    def getObjectRotation(self):
        """Berechnet die aktuelle Rotationsposition, je nach Winkel, des Objektes."""
        self.x = np.sin(self.angle * self.drc) * self.distance - np.cos(self.angle * self.drc) * 0.035  # x-Position
        self.y = -np.cos(self.angle * self.drc) * self.distance - np.sin(self.angle * self.drc) * 0.035  # y-Position
        self.z = 0.165  # z-Position
        self.angleradians = self.angle
        self.camerahead = 112.5  # Ausrichtung Kamerakopf (camera head)
        self.nValue = 0


    def correctRotation(self):
        """
        Berechnet die aktuellen Rotationskoordinaten des Objektes, um diese zu normalisieren, sprich zum Mullpunkt
        hin auszurichten. Dies ist notwendig, um die Pointcloud zusammensetzen zu können.
        """
        self.angleradians = self.angleradians * self.drc  # angle in Bogenmaß
        self.camerahead = (-self.camerahead) * self.drc  # Kamerakopf in Bogenmaß+Ausrichtung nach unten
        self.nValue = self.nValue * self.drc  # Nullwert zur Berechnung der Rotationsmatrix
        self.rotMatrix = [[np.cos(self.angleradians) * np.cos(self.nValue) - np.cos(self.camerahead) * np.sin(self.angleradians) * np.sin(self.nValue),
                           -np.cos(self.angleradians) * np.sin(self.nValue) - np.cos(self.camerahead) * np.cos(self.nValue) * np.sin(self.angleradians),
                           np.sin(self.angleradians) * np.sin(self.camerahead)],
                          [np.cos(self.nValue) * np.sin(self.angleradians) + np.cos(self.angleradians) * np.cos(self.camerahead) * np.sin(self.nValue),
                           np.cos(self.angleradians) * np.cos(self.camerahead) * np.cos(self.nValue) - np.sin(self.angleradians) * np.sin(self.nValue),
                           -np.cos(self.angleradians) * np.sin(self.camerahead)],
                          [np.sin(self.camerahead) * np.sin(self.nValue), np.cos(self.nValue) * np.sin(self.camerahead), np.cos(self.camerahead)]]


    def createSTLs(self, pType: str):
        '''
        Erstellt aus den vorherigen Aufnahmen die STL Datei des Objektes.
        Weitere Details: https://www.open3d.org/docs/latest/tutorial/geometry/pointcloud_outlier_removal.html
        Alle Beschreibungen der einzelnen Berechnungsmethoden stammen von folgendem Link:
         https://www.open3d.org/docs/latest/tutorial/Advanced/surface_reconstruction.html
        :param pType: Welche Berechnunsmethode? a = Alpha, b = Ball Pivot, p = Poisson
        :return: Das erstellte STL-Objekt.
        '''
        if pType == 'a':
            """
            Alpha shapes are a generalization of the convex hull. With decreasing alpha value the shape schrinks and 
            creates cavities. See Edelsbrunner and Muecke, “Three-Dimensional Alpha Shapes”, 1994.
            pcd (open3d.geometry.PointCloud) – PointCloud from which the TriangleMesh surface is reconstructed.
            alpha (float) – Parameter to control the shape. A very big value will give a shape close to the convex hull.
            """
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(scanner.getPointcloud(), 0.007)
            mesh.scale(1, center=(0, 0, 0))
            mesh.compute_vertex_normals()
            return mesh
        elif pType == 'b':
            """
            Function that computes a triangle mesh from a oriented PointCloud. This implements the Ball Pivoting
            algorithm proposed in F. Bernardini et al., “The ball-pivoting algorithm for surface reconstruction”, 1999.
            The implementation is also based on the algorithms outlined in Digne, “An Analysis and Implementation of a 
            Parallel Ball Pivoting Algorithm”, 2014. The surface reconstruction is done by rolling a ball with a given 
            radius over the point cloud, whenever the ball touches three points a triangle is created.
            pcd (open3d.geometry.PointCloud) – PointCloud from which the TriangleMesh surface is reconstructed. 
            Has to contain normals.
            radii (open3d.utility.DoubleVector) – The radii of the ball that are used for the surface reconstruction.
            """
            distances = self.complete_pcd.compute_nearest_neighbor_distance()
            avg_dist = np.mean(distances)
            radius = 1.5 * avg_dist  # Ansatz aus: https://stackoverflow.com/questions/56965268/how-do-i-convert-a-3d-point-cloud-ply-into-a-mesh-with-faces-and-vertices
            mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(scanner.getPointcloud(),o3d.utility.DoubleVector([radius, radius * 2]))
            mesh = mesh.filter_smooth_simple(number_of_iterations=10)
            mesh.scale(1, center=(0, 0, 0))
            mesh.compute_vertex_normals()
            return mesh
        elif pType == 'p':
            """
            The Poisson surface reconstruction method solves a regularized optimization problem to obtain a smooth
            surface. For this reason, Poisson surface reconstruction can be preferable to the methods mentioned above,
            as they produce non-smooth results since the points of the PointCloud are also the vertices of the resulting
            triangle mesh without any modifications. Open3D implements the method create_from_point_cloud_poisson which
            is basically a wrapper of the code of Kazhdan. An important parameter of the function is depth that defines
            the depth of the octree used for the surface reconstruction and hence implies the resolution of the
            resulting triangle mesh. A higher depth value means a mesh with more details.
            """
            mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(scanner.getPointcloud(),
                                                                                        depth=12)
            mesh.scale(1, center=(0, 0, 0))
            mesh.compute_vertex_normals()
            return mesh
        else:
            print("Falsche Eingabe für pType! Es sind nur a, b und p erlaubt.")
            return None


    def scan(self):
        if self.startPipeline():
            for i in range(0,self.numberScans):
                scanner.takeFoto()
                scanner.processFoto(int(i*self.apr))
                cv2.imwrite('./out' + str(i) + '.png', scanner.getImageArray())
                print("Jetzt drehen!")
                time.sleep(2)  # TODO Final höher setzen
            self.stopPipeline()
        o3d.visualization.draw_geometries([self.getPointcloud()])  # TODO Final entfernen
        o3d.io.write_point_cloud("model.ply", self.getPointcloud())

        o3d.io.write_triangle_mesh("model_poisson.stl", self.createSTLs('p'))
        o3d.io.write_triangle_mesh("model_alpha.stl", self.createSTLs('a'))
        o3d.io.write_triangle_mesh("model_ball_pivot.stl", self.createSTLs('b'))

if __name__ == '__main__':
    scanner = Scan(2 ,'c', 'y')
    scanner.scan()

