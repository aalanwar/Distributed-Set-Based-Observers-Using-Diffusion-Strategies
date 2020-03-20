function logVar(filename, varname, value)
    fid=fopen(filename,'a+');
    fprintf(fid,'\n');
    fprintf(fid,'%s=[',varname);
    [r c] = size(value);
    for i=1:r
        for j=1:c
            fprintf(fid,'%f ', value(i,j));
        end
        if i ~= r 
            fprintf(fid,';');
        end
    end
    fprintf(fid,'];');
    fprintf(fid,'\n');
    fclose(fid);
end