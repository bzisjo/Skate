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

% Last Modified by GUIDE v2.5 26-Apr-2017 20:13:49

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

%test
t = 0:.01:pi;
y = sin(t);
plot(t, y)


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

handles.skate_data = importdata(fullfile(pathname, filename));


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
