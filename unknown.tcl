#############################################################################
# Generated by PAGE version 6.0.1
#  in conjunction with Tcl version 8.6
#  Jan 05, 2021 07:34:09 PM -03  platform: Windows NT
set vTcl(timestamp) ""
if {![info exists vTcl(borrow)]} {
    tk_messageBox -title Error -message  "You must open project files from within PAGE."
    exit}


if {!$vTcl(borrow) && !$vTcl(template)} {

set vTcl(actual_gui_font_dft_desc)  TkDefaultFont
set vTcl(actual_gui_font_dft_name)  TkDefaultFont
set vTcl(actual_gui_font_text_desc)  TkTextFont
set vTcl(actual_gui_font_text_name)  TkTextFont
set vTcl(actual_gui_font_fixed_desc)  TkFixedFont
set vTcl(actual_gui_font_fixed_name)  TkFixedFont
set vTcl(actual_gui_font_menu_desc)  TkMenuFont
set vTcl(actual_gui_font_menu_name)  TkMenuFont
set vTcl(actual_gui_font_tooltip_desc)  TkDefaultFont
set vTcl(actual_gui_font_tooltip_name)  TkDefaultFont
set vTcl(actual_gui_font_treeview_desc)  TkDefaultFont
set vTcl(actual_gui_font_treeview_name)  TkDefaultFont
set vTcl(actual_gui_bg) #d9d9d9
set vTcl(actual_gui_fg) #000000
set vTcl(actual_gui_analog) #ececec
set vTcl(actual_gui_menu_analog) #ececec
set vTcl(actual_gui_menu_bg) #d9d9d9
set vTcl(actual_gui_menu_fg) #000000
set vTcl(complement_color) #d9d9d9
set vTcl(analog_color_p) #d9d9d9
set vTcl(analog_color_m) #ececec
set vTcl(active_fg) #000000
set vTcl(actual_gui_menu_active_bg)  #ececec
set vTcl(actual_gui_menu_active_fg)  #000000
set vTcl(pr,autoalias) 1
set vTcl(pr,relative_placement) 1
set vTcl(mode) Absolute
}




proc vTclWindow.top44 {base} {
    global vTcl
    if {$base == ""} {
        set base .top44
    }
    if {[winfo exists $base]} {
        wm deiconify $base; return
    }
    set top $base
    ###################
    # CREATING WIDGETS
    ###################
    vTcl::widgets::core::toplevel::createCmd $top -class Toplevel \
        -background $vTcl(actual_gui_bg) 
    wm focusmodel $top passive
    wm geometry $top 600x350
    update
    # set in toplevel.wgt.
    global vTcl
    global img_list
    set vTcl(save,dflt,origin) 1
    wm maxsize $top 1540 845
    wm minsize $top 120 1
    wm overrideredirect $top 0
    wm resizable $top 0 0
    wm deiconify $top
    wm title $top "Arm Window"
    vTcl:DefineAlias "$top" "TopLevel" vTcl:Toplevel:WidgetProc "" 1
    set vTcl(real_top) {}
    vTcl:withBusyCursor {
    ttk::label $top.tLa46 \
        -background $vTcl(actual_gui_bg) -foreground $vTcl(actual_gui_fg) \
        -font {-family {Segoe UI} -size 18 -weight bold -slant roman -underline 0 -overstrike 0} \
        -relief flat -anchor center -justify center \
        -text {Defining pHome for Manipulator} 
    vTcl:DefineAlias "$top.tLa46" "TLabel1" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.tLa46
    ttk::label $top.tLa47 \
        -background $vTcl(actual_gui_bg) -foreground $vTcl(actual_gui_fg) \
        -font {-family {Segoe UI} -size 13 -weight bold -slant italic -underline 0 -overstrike 0} \
        -relief flat -anchor center -justify left -text Increment: 
    vTcl:DefineAlias "$top.tLa47" "TLabel2" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.tLa47
    button $top.but49 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 12 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text 100 
    vTcl:DefineAlias "$top.but49" "Button100" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but49
    button $top.but51 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 12 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text 1 
    vTcl:DefineAlias "$top.but51" "Button1" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but51
    button $top.but52 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 12 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text 10 
    vTcl:DefineAlias "$top.but52" "Button10" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but52
    ttk::progressbar $top.tPr53 \
        -length 270 -maximum 1023 -value 50.0 -cursor crosshair 
    vTcl:DefineAlias "$top.tPr53" "TProgressbarM0" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.tPr53
    ttk::progressbar $top.tPr55 \
        -length 270 -maximum 1023 -value 900.0 -cursor crosshair 
    vTcl:DefineAlias "$top.tPr55" "TProgressbarM1" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.tPr55
    ttk::progressbar $top.tPr56 \
        -length 270 -maximum 1023 -value 512.0 -cursor fleur 
    vTcl:DefineAlias "$top.tPr56" "TProgressbarM2" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.tPr56
    button $top.but61 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˃ 
    vTcl:DefineAlias "$top.but61" "ButtonM02" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but61
    button $top.but62 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˃ 
    vTcl:DefineAlias "$top.but62" "ButtonM12" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but62
    button $top.but63 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˃ 
    vTcl:DefineAlias "$top.but63" "ButtonM22" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but63
    button $top.but65 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˂ 
    vTcl:DefineAlias "$top.but65" "ButtonM21" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but65
    button $top.but66 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˂ 
    vTcl:DefineAlias "$top.but66" "ButtonM01" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but66
    button $top.but67 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text ˂ 
    vTcl:DefineAlias "$top.but67" "ButtonM11" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but67
    button $top.but68 \
        -activebackground $vTcl(analog_color_m) -activeforeground #000000 \
        -background #25d8d3 -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 14 -weight bold -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -pady 0 -text DONE 
    vTcl:DefineAlias "$top.but68" "ButtonDone" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.but68
    label $top.lab70 \
        -background $vTcl(actual_gui_bg) -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 11 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) -text {M0: 50} 
    vTcl:DefineAlias "$top.lab70" "LabelM0" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.lab70
    label $top.lab71 \
        -activebackground #f9f9f9 -activeforeground black \
        -background $vTcl(actual_gui_bg) -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 11 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -text {M2: 512} 
    vTcl:DefineAlias "$top.lab71" "LabelM2" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.lab71
    label $top.lab72 \
        -activebackground #f9f9f9 -activeforeground black \
        -background $vTcl(actual_gui_bg) -disabledforeground #a3a3a3 \
        -font {-family {Segoe UI} -size 11 -weight normal -slant roman -underline 0 -overstrike 0} \
        -foreground $vTcl(actual_gui_fg) \
        -highlightbackground $vTcl(actual_gui_bg) -highlightcolor black \
        -text {M1: 900} 
    vTcl:DefineAlias "$top.lab72" "LabelM1" vTcl:WidgetProc "TopLevel" 1
    vTcl:copy_lock $top.lab72
    ###################
    # SETTING GEOMETRY
    ###################
    place $top.tLa46 \
        -in $top -x 0 -y 10 -width 600 -relwidth 0 -height 40 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.tLa47 \
        -in $top -x 442 -y 70 -width 105 -relwidth 0 -height 39 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.but49 \
        -in $top -x 455 -y 110 -width 77 -relwidth 0 -height 34 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.but51 \
        -in $top -x 455 -y 220 -width 77 -height 34 -anchor nw \
        -bordermode ignore 
    place $top.but52 \
        -in $top -x 455 -y 165 -width 77 -relwidth 0 -height 34 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.tPr53 \
        -in $top -x 90 -y 115 -width 270 -relwidth 0 -height 22 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.tPr55 \
        -in $top -x 90 -y 170 -width 270 -height 22 -anchor nw \
        -bordermode ignore 
    place $top.tPr56 \
        -in $top -x 90 -y 225 -width 270 -height 22 -anchor nw \
        -bordermode ignore 
    place $top.but61 \
        -in $top -x 370 -y 110 -width 30 -relwidth 0 -height 32 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.but62 \
        -in $top -x 370 -y 165 -width 30 -height 32 -anchor nw \
        -bordermode ignore 
    place $top.but63 \
        -in $top -x 370 -y 220 -width 30 -height 32 -anchor nw \
        -bordermode ignore 
    place $top.but65 \
        -in $top -x 50 -y 220 -width 30 -height 32 -anchor nw \
        -bordermode ignore 
    place $top.but66 \
        -in $top -x 50 -y 110 -width 30 -height 32 -anchor nw \
        -bordermode ignore 
    place $top.but67 \
        -in $top -x 50 -y 165 -width 30 -height 32 -anchor nw \
        -bordermode ignore 
    place $top.but68 \
        -in $top -x 270 -y 290 -width 83 -relwidth 0 -height 34 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.lab70 \
        -in $top -x 90 -y 99 -width 272 -relwidth 0 -height 16 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.lab71 \
        -in $top -x 90 -y 209 -width 272 -relwidth 0 -height 16 -relheight 0 \
        -anchor nw -bordermode ignore 
    place $top.lab72 \
        -in $top -x 90 -y 154 -width 272 -relwidth 0 -height 16 -relheight 0 \
        -anchor nw -bordermode ignore 
    } ;# end vTcl:withBusyCursor 

    vTcl:FireEvent $base <<Ready>>
}

set btop ""
if {$vTcl(borrow)} {
    set btop .bor[expr int([expr rand() * 100])]
    while {[lsearch $btop $vTcl(tops)] != -1} {
        set btop .bor[expr int([expr rand() * 100])]
    }
}
set vTcl(btop) $btop
Window show .
Window show .top44 $btop
if {$vTcl(borrow)} {
    $btop configure -background plum
}

