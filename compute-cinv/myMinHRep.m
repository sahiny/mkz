function [P_red] = myMinHRep(P)
	try	
		H_s = struct('A', P.A, 'B', P.b);
		[H_red] = cddmex('reduce_h', H_s);
		P_red = Polyhedron('A', H_red.A, 'b', H_red.B);
	catch
		disp('warning: reverting to LP minHRep')
		P_red = P.minHRep;
	end
end

