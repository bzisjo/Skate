function varargout = Skate_GUI(varargin)
% SKATE_GUI MATLAB code for Skate_GUI.fig
%      SKATE_GUI, by itself, creates a new SKATE_GUI or raises the existing
%      singleton*.
%
%      H = SKATE_GUI returns the handle to a new SKATE_GUI or the handle to
%      the existing singleton*.
%
%      SKATE_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SKATE_GUI.M with the given input arguments.
%
%      SKATE_GUI('Property','Value',...) creates a new SKATE_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Skate_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Skate_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Skate_GUI

% Last Modified by GUIDE v2.5 26-Apr-2017 23:16:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Skate_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @Skate_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before Skate_GUI is made visible.
function Skate_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Skate_GUI (see VARARGIN)

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Skate_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Skate_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in load_button.
function load_button_Callback(hObject, eventdata, handles)
% hObject    handle to load_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

[filename, pathname] = uigetfile('*.txt', 'Select Skate Data to Load');

if isequal(filename, 0)
    disp('User selected cancel')
else
    disp(['User selected ', fullfile(pathname, filename)])
end

test_data = importdata(fullfile(pathname, filename));
session1_ind = find(test_data(:,1) == 0);
handles.session1_data = test_data(session1_ind(1):session1_ind(length(session1_ind)),:);

session2_ind = find(test_data(:,1) == 1);
handles.session2_data = test_data(session2_ind(1):session2_ind(length(session2_ind)),:);

handles.data = test_data;

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in test_button.
function test_button_Callback(hObject, eventdata, handles)
% hObject    handle to test_button (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
disp(handles.skate_data)

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --------------------------------------------------------------------
function uibuttongroup1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to uibuttongroup1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
temp = get(hObject, 'Value');
disp(temp);


% --- Executes on button press in pushbutton3.
function pushbutton3_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in force_radio.
function force_radio_Callback(hObject, eventdata, handles)
% hObject    handle to force_radio (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of force_radio
%handles.analysis = 12;
%disp(handles.hi)

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes when selected object is changed in data_displayed.
function data_displayed_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in data_displayed 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

data_type = get(hObject, 'String');
switch data_type
    case 'Force'
        %checks if filtered analysis type choosen
        if strcmp(handles.analysis_type, 'Filtered')
            fsr1_plot = abs(handles.fsr1);
            fsr2_plot = abs(handles.fsr2);
        else
            fsr1_plot = handles.fsr1;
            fsr2_plot = handles.fsr2;
        end
        
        plot(handles.left_graph, 1:length(handles.fsr1), fsr1_plot);
        title(handles.left_graph, 'Force from Toe FSR')
        ylabel(handles.left_graph, 'lbs')

        plot(handles.middle_graph, 1:length(handles.fsr2), fsr2_plot);
        title(handles.middle_graph, 'Force from Heel FSR')
        ylabel(handles.middle_graph, 'lbs')
        
        cla(handles.right_graph, 'reset');   %clears right figure
    case 'Acceleration'
        %checks if filtered analysis type choosen
        if strcmp(handles.analysis_type, 'Filtered')
            left_plot = abs(handles.accel(:,1));
            middle_plot = abs(handles.accel(:,2));
            right_plot = abs(handles.accel(:,3));
        else
            left_plot = handles.accel(:,1);
            middle_plot = handles.accel(:,2);
            right_plot = handles.accel(:,3);
        end
        
        plot(handles.left_graph, 1:length(handles.accel(:,1)), left_plot);
        title(handles.left_graph, 'X Axis Accelerometer Data')
        ylabel(handles.left_graph, 'gravity (g)')

        plot(handles.middle_graph, 1:length(handles.accel(:,2)), middle_plot);
        title(handles.middle_graph, 'Y Axis Accelerometer Data')
        ylabel(handles.middle_graph, 'gravity (g)')
        
        plot(handles.right_graph, 1:length(handles.accel(:,3)), right_plot);
        title(handles.middle_graph, 'Z Axis Accelerometer Data')
        ylabel(handles.middle_graph, 'gravity (g)')
    case 'Angles'
        %checks if filtered analysis type choosen
        if strcmp(handles.analysis_type, 'Filtered')
            left_plot = abs(handles.angle(:,1));
            middle_plot = abs(handles.angle(:,2));
            right_plot = abs(handles.angle(:,3));
        else
            left_plot = handles.angle(:,1);
            middle_plot = handles.angle(:,2);
            right_plot = handles.angle(:,3);
        end
        
        plot(handles.left_graph, 1:length(handles.angle(:,1)), left_plot);
        plot(handles.middle_graph, 1:length(handles.angle(:,2)), middle_plot);
        plot(handles.right_graph, 1:length(handles.angle(:,3)), right_plot);
        
        % labeling based on analysis type choosen
        if strcmp(handles.analysis_type, 'Raw')
            title(handles.left_graph, 'X Axis Rotational Speed')
            ylabel(handles.left_graph, 'degrees/second')
            title(handles.middle_graph, 'Y Axis Rotational Speed')
            ylabel(handles.middle_graph, 'degrees/second')
            title(handles.right_graph, 'Z Axis Rotational Speed')
            ylabel(handles.right_graph, 'degrees/second')
        else
            title(handles.left_graph, 'X Axis Rotation')
            ylabel(handles.left_graph, 'degrees')
            title(handles.middle_graph, 'Y Axis Rotation')
            ylabel(handles.middle_graph, 'degrees')
            title(handles.right_graph, 'Z Axis Rotation')
            ylabel(handles.right_graph, 'degrees')
        end
end

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes on selection change in analysis_type.
function analysis_type_Callback(hObject, eventdata, handles)
% hObject    handle to analysis_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns analysis_type contents as cell array
%        contents{get(hObject,'Value')} returns selected item from analysis_type

contents = cellstr(get(hObject,'String'));
disp(contents{get(hObject,'Value')});
curr_anly = contents{get(hObject,'Value')};
handles.current_analysis = curr_anly;

if isempty(handles.data)
    disp('Load File First')
else
    if strcmp(curr_anly, 'Raw')
        handles.analysis_type = 'Raw';
        handles.fsr1 = convertFSR(handles.session2_data(:,2));
        handles.fsr2 = convertFSR(handles.session2_data(:,3));
        
        handles.angle = handles.session2_data(:,7:9);
        handles.accel = handles.session2_data(:,4:6);
        
        % initialized plotting 
        plot(handles.left_graph, handles.fsr1);
        title(handles.left_graph, 'Force from Toe FSR')
        ylabel(handles.left_graph, 'lbs')

        plot(handles.middle_graph, handles.fsr2);
        title(handles.middle_graph, 'Force from Heel FSR')
        ylabel(handles.middle_graph, 'lbs')

        cla(handles.right_graph, 'reset');   %clears right figure
        set(handles.data_displayed,'selectedobject',handles.force_radio)
    elseif strcmp(curr_anly, 'Integrated')
        handles.analysis_type = 'Integrated';
        handles.fsr1 = convertFSR(handles.session2_data(:,2));
        handles.fsr2 = convertFSR(handles.session2_data(:,3));
        
        handles.accel = handles.session2_data(:,4:6);
        
        dps = handles.session2_data(:,7:9);
        angles_int = [rk_integrator(dps(:,1)/60); rk_integrator(-dps(:,2)/60); rk_integrator(dps(:,3)/60)];
        angles_int = (angles_int).';
        handles.angle = angles_int;
        
        % initialized plotting 
        plot(handles.left_graph, handles.fsr1);
        title(handles.left_graph, 'Force from Toe FSR')
        ylabel(handles.left_graph, 'lbs')

        plot(handles.middle_graph, handles.fsr2);
        title(handles.middle_graph, 'Force from Heel FSR')
        ylabel(handles.middle_graph, 'lbs')

        cla(handles.right_graph, 'reset');   %clears right figure
        set(handles.data_displayed,'selectedobject',handles.force_radio)
    elseif strcmp(curr_anly, 'Filtered')
        handles.analysis_type = 'Filtered';
        FSR1 = convertFSR(handles.session2_data(:,2));
        FSR2 = convertFSR(handles.session2_data(:,3));
        accel = handles.session2_data(:,4:6);
        dps = handles.session2_data(:,7:9);
        
        % low pass filtering algorithm
        
        cutoff = 20;
        accel_filt = [rect_lpf(accel(:,1),cutoff) rect_lpf(accel(:,2),cutoff) rect_lpf(accel(:,3),cutoff)];
        %accel_filt = accel_filt.';
        handles.accel = accel_filt;
        disp(size(accel_filt));
        
        dps_filt = [rect_lpf(dps(:,1),cutoff) rect_lpf(dps(:,2),cutoff) rect_lpf(dps(:,3),cutoff)];
        %dps_filt = dps_filt.';
        disp(size(dps_filt));
        
        handles.fsr1 = rect_lpf(FSR1, cutoff);
        handles.fsr2 = rect_lpf(FSR2, cutoff);
                
        % integrate
        angles_int = [rk_integrator(dps(:,1)/60); rk_integrator(-dps(:,2)/60); rk_integrator(dps(:,3)/60)];
        angles_int = (angles_int).';
        handles.angle = angles_int;
        
        % complimentary filter
        angle_accelX= (atan(accel_filt(:,2) ./ sqrt(accel_filt(:,1).^2 + accel_filt(:,3).^2)))*180/pi;
        angle_accelY = (- atan(accel_filt(:,1) ./ sqrt(accel_filt(:,2).^2 + accel_filt(:,3).^2)))*180/pi;
        
        alpha = .9;
        angleX_filt = (1-alpha) * angles_int(:,1) + alpha .* angle_accelX;
        angleY_filt = (1-alpha) * angles_int(:,2) + alpha .* angle_accelY;
        disp(size(angleX_filt))
        disp(size(angles_int))
        
        handles.angle = [angleX_filt angleY_filt angles_int(:,3)];
        
        % initialized plotting 
        plot(handles.left_graph, 1:length(handles.fsr1), abs(handles.fsr1));
        title(handles.left_graph, 'Force from Toe FSR')
        ylabel(handles.left_graph, 'lbs')

        plot(handles.middle_graph, 1:length(handles.fsr2), abs(handles.fsr2));
        title(handles.middle_graph, 'Force from Heel FSR')
        ylabel(handles.middle_graph, 'lbs')
        
        cla(handles.right_graph, 'reset');   %clears right figure
        set(handles.data_displayed,'selectedobject',handles.force_radio)
    end
end

% Choose default command line output for Skate_GUI
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function analysis_type_CreateFcn(hObject, eventdata, handles)
% hObject    handle to analysis_type (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
