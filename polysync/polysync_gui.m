function varargout = polysync_gui(varargin)
% POLYSYNC_GUI MATLAB code for polysync_gui.fig
%      POLYSYNC_GUI, by itself, creates a new POLYSYNC_GUI or raises the existing
%      singleton*.
%
%      H = POLYSYNC_GUI returns the handle to a new POLYSYNC_GUI or the handle to
%      the existing singleton*.
%
%      POLYSYNC_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POLYSYNC_GUI.M with the given input arguments.
%
%      POLYSYNC_GUI('Property','Value',...) creates a new POLYSYNC_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before polysync_gui_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to polysync_gui_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help polysync_gui

% Last Modified by GUIDE v2.5 06-Sep-2017 18:36:16

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @polysync_gui_OpeningFcn, ...
                   'gui_OutputFcn',  @polysync_gui_OutputFcn, ...
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


% --- Executes just before polysync_gui is made visible.
function polysync_gui_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to polysync_gui (see VARARGIN)

% Choose default command line output for polysync_gui
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes polysync_gui wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = polysync_gui_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes on button press in button_enable.
function button_enable_Callback(hObject, eventdata, handles)
% hObject    handle to button_enable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in button_disable.
function button_disable_Callback(hObject, eventdata, handles)
% hObject    handle to button_disable (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
