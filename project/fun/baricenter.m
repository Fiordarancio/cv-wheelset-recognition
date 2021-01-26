% evaluate baricenter 
function bar = baricenter(ptcloud)
    bar = zeros(1,3);
    for i=1 : ptcloud.Count
        bar(1) = bar(1) + ptcloud.Location(i,1);
        bar(2) = bar(2) + ptcloud.Location(i,2);
        bar(3) = bar(3) + ptcloud.Location(i,3);
    end
    bar = bar / ptcloud.Count;
end