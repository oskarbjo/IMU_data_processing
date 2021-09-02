function [ind] = findPeakIndices(a)
[b,ind]=findpeaks(a,'MinPeakProminence',4);
end

