classdef Bread < handle
	properties
		faces;
		points;
		data;
		numPoints;
		bread_h;
	end

	methods
		function self = Bread(startTr)
			[faces, points, data] = getPolyData(self);
			self.faces = faces;
			self.points = points;
			self.data = data;
			numPoints = size(points);
			self.numPoints = numPoints(1);
			self.bread_h = initBread(self, faces, points);
            self.updatePos(startTr);
            drawnow();
        end
		
		function [faces, points, data] = getPolyData(self)
			[faces, points, data] = plyread("bread.ply", "tri");
		end

		function bread_h = initBread(self, faces, points)
			hold on
			bread_h=trisurf(faces,points(:,1),points(:,2),points(:,3),"LineStyle","none","Facecolor",[198 137 88]/255);
			hold off
		end
		
		function updatePos(self, breadTr)
			for j=1:self.numPoints
				self.bread_h.Vertices(j,:)=transl(breadTr*transl(self.points(j,:)))';
			end
		end
		
	end
end
