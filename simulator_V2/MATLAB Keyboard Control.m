function [lin_vel, ang_vel] = get_keyboard_command()
% Settings
lin_vel_inc = 0.01;
ang_vel_inc = 0.01;

% Initialise to zero
lin_vel = 0;
ang_vel = 0;

% Get keyboard input
fig = figure('Color','white', 'Menu','none','KeyPressFcn',@Key_Down);
while ishandle(fig)
    clf(fig);
    display_velocities();
    axis off
    drawnow;
end

function display_velocities()
    text(0.5,0.75, sprintf("u: %.2f m/s", lin_vel), ...
        'FontSize', 50, 'HorizontalAlignment','Center', 'VerticalAlignment','Middle');
    
    text(0.5,0.25, sprintf("q: %.2f rad/s", ang_vel), ...
        'FontSize', 50, 'HorizontalAlignment','Center', 'VerticalAlignment','Middle'); 
end


function Key_Down(src,event)
    switch event.Key
        case 'uparrow'
          lin_vel = lin_vel + lin_vel_inc;
        case 'downarrow'
          lin_vel = lin_vel - lin_vel_inc;
        case 'leftarrow'
          ang_vel = ang_vel + ang_vel_inc;
        case 'rightarrow'
          ang_vel = ang_vel - ang_vel_inc;
        case 'space'
          close(src);
    end    
end

end
