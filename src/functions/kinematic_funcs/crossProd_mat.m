function cross_prod_mat = crossProd_mat(vec)
v1 = vec(1);
v2 = vec(2);
v3 = vec(3);

cross_prod_mat = [0 -v3 v2;...
    v3 0 -v1;...
    -v2 v1 0];
end

