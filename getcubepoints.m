function P = getcubepoints()
 A = [0 0 0];
 B = [1 0 0];
 C = [0 1 0];
 D = [0 0 1];
 E = [0 1 1];
 F = [1 0 1];
 G = [1 1 0];
 H = [1 1 1];
 P = [A;B;F;H;G;C;A;D;E;H;F;D;E;C;G;B]-[0.5,0.5,0.5];
end

