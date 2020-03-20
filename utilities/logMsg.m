function logMsg(filename, Msg, varargin )
    fid=fopen(filename,'a+');
    if length(varargin) == 0 
        fprintf(fid,'\n');
        fprintf(fid,Msg);
        fprintf(fid,'\n');
    else
        fprintf(fid,'\n');
        fprintf(fid,Msg,varargin{:});
        fprintf(fid,'\n');
    end
    fclose(fid);
end