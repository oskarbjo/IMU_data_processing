function [ind] = findPeakIndices(a)
[b,ind]=findpeaks(a,'MinPeakProminence',5);
end

