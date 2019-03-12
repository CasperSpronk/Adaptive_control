annotations = find_system('Bixler/VISUAL INTERFACE','FindAll','on','Type','annotation');
% names = get_param(annotations,'Name');


init_cond_text = strcat('Position[m]:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[',num2str(Init.xyz),']', '<br>',...
                        'Velocity[m/s]:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[',num2str(Init.uvw),']', '<br>',...
                        'Attitude[deg]:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[',num2str(Init.attitude),']', '<br>',...
                        'Angular rates[deg/s]:&nbsp;[',num2str(Init.pqr),']', '<br><br>',...
                        'W. amplitude[m/s]:&nbsp;&nbsp;&nbsp;&nbsp;[',num2str(Init.windConst),']','<br>');

% 'Attitude[deg]: [',num2str(Init.attitude),']', '<br>'

set_param(annotations(16),'Name', init_cond_text);

clear annotations init_cond_text