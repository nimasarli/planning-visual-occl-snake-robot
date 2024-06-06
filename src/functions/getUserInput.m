function userInput = getUserInput()
while true
userInput = input('Right(r) or Left(l) jaw? (r/l) ','s');
if strcmp(userInput,'r') || strcmp(userInput,'l')
    break
end
disp('Enter r or l please!');
end

end
